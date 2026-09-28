"""Sample-aligned live adapter for the existing modulation estimators."""

import copy

import numpy as np

from .. import modulation, resonator


_CYCLE_FIELDS = ('detuning_hz', 'detuning_linewidths', 'dphi_df',
                 'd2phi_df2', 'curvature_over_slope')


class LiveDemodulator:
    """Demodulate tagged revisions with a stable reference within each epoch."""

    def __init__(self, method='fast', settling='none', linewidth_hz=None,
                 calibration=None, cycle_calibration=None):
        if method not in ('fast', 'accurate'):
            raise ValueError('demod_method must be fast or accurate')
        if settling not in ('none', 'raw'):
            raise ValueError('settling must be none or raw')
        if linewidth_hz is not None and (
                not np.all(np.isfinite(linewidth_hz)) or np.any(np.asarray(linewidth_hz) <= 0)):
            raise ValueError('linewidth_hz must be positive and finite')
        self.method = method
        self.settling = settling
        self.linewidth_hz = linewidth_hz
        self.calibration = calibration
        # Calibrations that circle-centre the I/Q for the per-cycle slope,
        # curvature and detuning, whatever the per-sample conversion is.
        self.cycle_calibration = cycle_calibration
        self.references = {}
        self.reference = None
        self.revision = None
        self.signature = None
        self.state_signature = None
        self._epoch_references = {}
        self._baseline_centers = {}

    def __call__(self, samples, tones=None):
        if samples.get('modulation_demodulated'):
            return samples
        keys = sorted(map(int, samples['i_data'])) if tones is None else sorted(set(tones))
        if not keys:
            raise ValueError('Select at least one tone for demodulation')
        n = int(samples['num_samples'])
        sample_index = np.asarray(samples.get('sample_index', np.arange(n)), dtype=np.int64)
        revision = np.asarray(samples['modulation_revision'], dtype=int)
        points = np.asarray(samples['modulation_point'], dtype=int)
        initial = modulation.active_modulation_state(samples.get('info'))
        events = []
        if initial is not None:
            initial_rev = initial.get('applied_revision', initial.get('revision'))
            if initial_rev is None:
                active_revs = revision[points > 0]
                initial_rev = int(active_revs[0]) if len(active_revs) else None
            events.append((-(1 << 62), None if initial_rev is None else
                           int(initial_rev) & 0x7fff, initial,
                           ('initial', initial_rev, repr(initial))))
        for order, record in enumerate(samples.get('tone_updates', ())):
            payload = record.get('payload', {})
            if payload.get('type') not in ('SNAPSHOT', 'TONE_UPDATE'):
                continue
            if payload.get('op') == 'tracking_dry_run':
                continue
            state = payload.get('modulation_state')
            rev = payload.get('revision')
            if state is None:
                if payload.get('type') == 'SNAPSHOT':
                    # Legacy snapshots carry only a compact summary. Preserve
                    # the initial metadata for frames tagged with its revision.
                    continue
                if payload.get('op') != 'disable':
                    # Old update records lack the offsets/RX correction needed
                    # to reconstruct the next revision.
                    continue
            events.append((int(record['sample_index']),
                           None if rev is None else int(rev) & 0x7fff,
                           state, ('update', int(record['sample_index']), rev)))
        events.sort(key=lambda item: item[0])
        if not events:
            raise ValueError('Demodulation requires modulation state in stream metadata')

        fields = ('resonance_shift_hz', 'frequency_hz', 'frequency_shift_hz',
                  'dissipation', *_CYCLE_FIELDS)
        merged = {field: np.full((n, len(keys)), np.nan) for field in fields}
        iq = {field: np.full((n, len(keys)), np.nan) for field in ('i_data', 'q_data')}
        z_demod = np.full((n, len(keys)), np.nan + 1j * np.nan)
        spectra_parts = []
        rates = []
        counts = 0
        covered = np.zeros(n, dtype=bool)
        for event_index, (begin, event_rev, state, epoch) in enumerate(events):
            if state is None:
                continue
            # Old-revision frames may still be queued after an announcement.
            # The tag on each IQ frame chooses the state; only a later
            # announcement reusing this same revision ID ends its epoch.
            stop = next((later[0] for later in events[event_index + 1:]
                         if later[1] == event_rev), 1 << 62)
            mask = ((sample_index >= begin) & (sample_index < stop)
                    & (points > 0))
            if event_rev is not None:
                mask &= revision == event_rev
            if not mask.any():
                continue
            covered |= mask
            global_idx = np.flatnonzero(mask)
            lo, hi = int(global_idx[0]), int(global_idx[-1]) + 1
            local_idx = global_idx - lo
            selected = {
                key: value[lo:hi] if isinstance(value, np.ndarray)
                and value.shape[:1] == (n,) else value
                for key, value in samples.items()}
            selected['num_samples'] = hi - lo
            selected['num_tones'] = len(keys)
            selected['modulation_point'] = np.where(mask[lo:hi], points[lo:hi], 0)
            for field in ('i_data', 'q_data'):
                by_id = {int(k): v for k, v in samples[field].items()}
                selected[field] = {f'{i:04d}': by_id[t][lo:hi]
                                   for i, t in enumerate(keys)}
            if 'stream_flags' in samples:
                selected['stream_flags'] = {
                    key: value[lo:hi]
                    for key, value in samples['stream_flags'].items()}
            local_state = copy.deepcopy(state)
            by_id = {int(t['index']): t for t in local_state['tones']}
            centers = {}
            for tone in keys:
                center = by_id[tone].get('center_hz')
                if center is not None:
                    centers[tone] = float(center)
                    self._baseline_centers.setdefault(tone, float(center))
            local_state['tones'] = [dict(by_id[t], index=i)
                                    for i, t in enumerate(keys)]
            for field in ('mod_indices', 'modulated_indices'):
                if field in local_state:
                    selected_ids = set(map(int, local_state[field]))
                    local_state[field] = [i for i, t in enumerate(keys)
                                          if t in selected_ids]
            tone_info = samples['info'].get('tones', {})
            selected['info'] = {'modulation': local_state, 'tones': {}}
            if 'frequencies_hz' in tone_info:
                selected['info']['tones']['frequencies_hz'] = (
                    np.asarray(tone_info['frequencies_hz'])[keys])
            for field in ('blind_indices', 'regular_indices'):
                if field in tone_info:
                    selected['info']['tones'][field] = [
                        i for i, t in enumerate(keys) if t in tone_info[field]]
            calibration = None
            if self.calibration is not None:
                calibration = {i: resonator.calibration_for_tone(self.calibration, t)
                               for i, t in enumerate(keys)}
                if any(cal is None for cal in calibration.values()):
                    raise ValueError(
                        'Mobius demodulation requires a calibration for every selected tone')
            linewidth = self.linewidth_hz
            if linewidth is not None and np.ndim(linewidth):
                linewidth = np.asarray(linewidth)[keys]
            cycle_calibration = None
            if self.cycle_calibration is not None:
                cycle_calibration = {
                    i: resonator.calibration_for_tone(self.cycle_calibration, t)
                    for i, t in enumerate(keys)}
            references = self._epoch_references.setdefault(epoch, {})
            known = [t in references for t in keys]
            self.reference = (np.column_stack([references[t] for t in keys])
                              if all(known) else None)
            result = self._demodulate(selected, calibration, cycle_calibration,
                                      linewidth)
            candidate = result.get('point_reference_z')
            if candidate is not None:
                reference = candidate.copy()
                for i, tone in enumerate(keys):
                    if tone in references:
                        reference[:, i] = references[tone]
                    elif np.all(np.isfinite(reference[:, i])):
                        references[tone] = reference[:, i].copy()
                if any(known) and not all(known):
                    self.reference = reference
                    result = self._demodulate(selected, calibration,
                                              cycle_calibration, linewidth)
            self.references = references
            self.state_signature = repr(state)
            for field in ('i_data', 'q_data'):
                for i in range(len(keys)):
                    iq[field][global_idx, i] = result[field][f'{i:04d}'][local_idx]
            for field in fields:
                if field in result:
                    merged[field][global_idx] = result[field][local_idx]
            # A new per-point reference starts at each revision. Restore the
            # applied centre steps so resonance-side frequency remains on the
            # viewer session's original frequency reference.
            for i, tone in enumerate(keys):
                if tone in centers:
                    delta = centers[tone] - self._baseline_centers[tone]
                    for field in ('resonance_shift_hz', 'frequency_shift_hz',
                                  'frequency_hz'):
                        merged[field][global_idx, i] += delta
            if 'z_demod' in result:
                z_demod[global_idx] = result['z_demod'][local_idx]
            spectra_parts.append(result['_live_cycle_spectra'])
            rates.append(result['_live_cycle_rate'])
            counts += int(result.get('_live_complete_cycles', 0))

        output = dict(samples)
        for field in ('i_data', 'q_data'):
            output[field] = {f'{tone:04d}': iq[field][:, i]
                             for i, tone in enumerate(keys)}
        for field, values in merged.items():
            output[field] = {f'{tone:04d}': values[:, i]
                             for i, tone in enumerate(keys)}
        output['z_demod'] = z_demod
        output['modulation_demodulated'] = True
        if rates:
            common_rate = all(np.isclose(rate, rates[0]) for rate in rates)
            selected_parts = spectra_parts if common_rate else spectra_parts[-1:]
            output['_live_cycle_rate'] = rates[0] if common_rate else rates[-1]
            spectra = {}
            for field in _CYCLE_FIELDS:
                rows = [part[field] for part in selected_parts if field in part]
                if rows:
                    gap = np.full((1, len(keys)), np.nan)
                    combined = np.concatenate(
                        [piece for i, row in enumerate(rows)
                         for piece in ((gap, row) if i else (row,))])
                else:
                    combined = np.empty((0, len(keys)))
                spectra[field] = {f'{tone:04d}': combined[:, i]
                                  for i, tone in enumerate(keys)}
            output['_live_cycle_spectra'] = spectra
        else:
            output['_live_cycle_rate'] = float(samples['sample_rate'])
            output['_live_cycle_spectra'] = {
                field: {f'{tone:04d}': np.empty(0) for tone in keys}
                for field in _CYCLE_FIELDS}
        missing = int(np.count_nonzero((points > 0) & ~covered))
        output['_live_demod_note'] = (
            f'Demodulated ({self.method}, settling={self.settling}); '
            f'{len(spectra_parts)} revision segment(s); '
            f'{missing} samples awaiting revision metadata')
        if rates and not all(np.isclose(rate, rates[0]) for rate in rates):
            output['_live_demod_note'] += '; spectra use latest cadence'
        if (self.calibration is None and self.linewidth_hz is None
                and self.cycle_calibration is None):
            output['_live_demod_note'] += '; detuning needs linewidth_hz or calibrations'
        return output

    def _demodulate(self, samples, calibration, cycle_calibration, linewidth_hz):
        if samples.get('modulation_demodulated'):
            return samples
        state = modulation.active_modulation_state(samples.get('info'))
        if state is None:
            raise ValueError('Demodulation requires modulation state in stream metadata')
        points = np.asarray(samples['modulation_point'])
        n = len(points)
        tones = int(samples['num_tones'])
        N, spp = int(state['num_points']), int(state['samples_per_point'])
        settle = int(state.get('n_settle', 0))
        if N < 2 or spp < 1 or not 0 <= settle < spp:
            raise ValueError('Demodulation requires >=2 probe points and non-settling dwell samples')
        signature = (N, spp, settle)
        revisions = np.asarray(samples['modulation_revision'])
        missing = np.asarray(samples.get('packet_missing', np.zeros(n)), dtype=bool)
        active = (points > 0) & ~missing
        rev = np.unique(revisions[active])
        if len(rev) > 1:
            raise ValueError('A demodulation segment must contain one revision')
        if len(rev):
            expected = state.get('applied_revision', state.get('revision'))
            if expected is not None and (int(expected) & 0x7fff) != rev[0]:
                raise ValueError('Stream modulation revision disagrees with metadata')
            self.revision = int(rev[0])
            self.signature = signature

        # Reject partial dwells/cycles before group_cycles can truncate unequal
        # dwell lengths. Keep original indices, including gaps, for demodulation.
        length = N * spp
        starts = np.flatnonzero((points == 1) & np.r_[True, points[:-1] != 1])
        pattern = np.repeat(np.arange(1, N + 1), spp)
        settling_pattern = np.tile(np.arange(spp) < settle, N)
        flags = np.asarray(samples['modulation_settling'])
        counter = np.asarray(samples['packet_counter'], dtype=np.int64)
        discontinuity = np.asarray(samples.get('packet_discontinuity', np.zeros(n)), dtype=bool)
        complete = np.zeros(n, dtype=bool)
        # One row per candidate cycle: the sample indices it would span.
        window = starts[starts + length <= n, None] + np.arange(length)
        step_ok = np.r_[(np.diff(counter) & 0xffffffff) == 1, False]
        ok = (np.all(points[window] == pattern, axis=1)
              & np.all(flags[window] == settling_pattern, axis=1)
              & ~np.any(missing[window], axis=1)
              & np.all(step_ok[window[:, :-1]], axis=1)
              & ~np.any(discontinuity[window], axis=1))
        complete[window[ok]] = True
        clean = dict(samples)
        clean['modulation_point'] = np.where(complete, points, 0)
        # Apply the state-specific RX correction to all dwell samples,
        # including settling samples retained by --settling raw.
        if not clean.get('readout_correction_applied', False):
            correction = modulation.readout_correction_factors(
                {'modulation': state}, tones)
            if correction is not None:
                selected = (points >= 1) & (points <= N)
                clean['i_data'] = dict(samples['i_data'])
                clean['q_data'] = dict(samples['q_data'])
                for tone in range(tones):
                    key = f'{tone:04d}'
                    z = (np.asarray(samples['i_data'][key], dtype=float)
                         + 1j * np.asarray(samples['q_data'][key], dtype=float))
                    z = z.copy()
                    z[selected] *= correction[points[selected] - 1, tone]
                    clean['i_data'][key] = z.real
                    clean['q_data'][key] = z.imag
                clean['readout_correction_applied'] = True
        # Time continuity is already checked above, including uint32 wrap.
        clean['packet_counter'] = np.arange(n)
        grouped = modulation.group_cycles(clean, state, reduce=None, on_missing='notify')
        count = len(grouped['z'])
        if count:
            result = modulation.demodulate_timestream(
                grouped, method=self.method, calibration=calibration,
                reference_z=self.reference, data_dict=clean,
                fill_settling=self.settling)
            if self.reference is None:
                reference = result['point_reference_z']
                if np.all(np.isfinite(reference)):
                    self.reference = reference.copy()
            cycle = modulation.demodulate(
                grouped, method=self.method, calibration=cycle_calibration,
                linewidth_hz=linewidth_hz)
            slope = cycle['dphi_df']
            cycle['curvature_over_slope'] = np.divide(
                cycle['d2phi_df2'], slope, out=np.full_like(slope, np.nan),
                where=np.isfinite(slope) & (slope != 0))
            # Detuning is an absolute probe-side estimate, not the negative of
            # a capture-relative resonance shift. Hold cycle estimates only in
            # the time display; spectra below retain the true cycle cadence.
            index = result['cycle_index']
            valid = complete & (index >= 0)
            if self.settling == 'none':
                valid &= flags == 0
            for field in _CYCLE_FIELDS:
                result[field] = np.full((n, tones), np.nan)
                result[field][valid] = cycle[field][index[valid]]
            cycle_indices = np.asarray(grouped['cycle_sample_index'], dtype=int)
            positions = np.rint((cycle_indices - cycle_indices[0]) / length).astype(int)
            if np.any(cycle_indices - cycle_indices[0] != positions * length):
                raise ValueError('Modulation cycle indices are not on a regular cadence')
            result['_live_cycle_spectra'] = {}
            for field in _CYCLE_FIELDS:
                rows = np.full((positions[-1] + 1, tones), np.nan)
                rows[positions] = cycle[field]
                result['_live_cycle_spectra'][field] = rows
            result['_live_cycle_rate'] = samples['sample_rate'] / length
        else:
            result = dict(samples)
            for field in ('resonance_shift_hz', 'dissipation', *_CYCLE_FIELDS):
                result[field] = np.full((n, tones), np.nan)
            result['i_data'] = {key: np.full(n, np.nan) for key in samples['i_data']}
            result['q_data'] = {key: np.full(n, np.nan) for key in samples['q_data']}
            result['_live_cycle_spectra'] = {field: np.empty((0, tones)) for field in
                                               _CYCLE_FIELDS}
            result['_live_cycle_rate'] = samples['sample_rate'] / length
        for field in ('sample_index', 'time_s', 'telescope_time', 'packet_counter',
                      'packet_missing', 'packet_discontinuity'):
            if field in samples:
                result[field] = samples[field]
        result['modulation_demodulated'] = True
        result['_live_complete_cycles'] = count
        result['_live_demod_note'] = (
            f'Demodulated ({self.method}, settling={self.settling}); '
            f'{count} complete cycles; slope/curvature/detuning held per cycle')
        if (self.calibration is None and self.linewidth_hz is None
                and self.cycle_calibration is None):
            result['_live_demod_note'] += '; detuning needs linewidth_hz or calibrations'
        return result
