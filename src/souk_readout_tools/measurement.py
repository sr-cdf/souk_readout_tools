"""
Parameter space measurement framework.

Provides tools for repeating readout measurements across an external
parameter (e.g. cryostat temperature, attenuator setting, bias voltage).

The external parameter is abstract — the user provides:
- A function to SET the parameter (for controllable parameters), or
- A function to READ the parameter (for monitored parameters), or
- Both.

Measurement types:
- ParameterSweep: step through parameter values, run a measurement at each
- TimedMeasurement: run measurements at fixed time intervals
- ConditionalMeasurement: run measurements when a condition is met

All measurements log the parameter value alongside the readout data.
"""

import time
import numpy as np
from dataclasses import dataclass, field
from typing import Callable, Optional, Any, List, Dict


@dataclass
class MeasurementPoint:
    """A single measurement point with parameter value and data."""
    parameter_value: Any
    parameter_name: str
    timestamp: float
    data: dict
    index: int


class ParameterSweep:
    """
    Sweep an external parameter, taking measurements at each value.

    Args:
        client: ReadoutClient instance.
        parameter_name: Human-readable name for the parameter (e.g.
            'temperature_mk', 'tx_attenuation_db').
        set_parameter: Callable that sets the parameter value. Called as
            set_parameter(value). Can be None for read-only parameters.
        get_parameter: Callable that reads the current parameter value.
            Called as get_parameter() -> value. Can be None if set_parameter
            is provided (value is assumed to be what was set).
        settle_time: Time in seconds to wait after setting the parameter
            before measuring. Default 0.
    """

    def __init__(self, client, parameter_name,
                 set_parameter=None, get_parameter=None,
                 settle_time=0.0):
        self.client = client
        self.parameter_name = parameter_name
        self.set_parameter = set_parameter
        self.get_parameter = get_parameter
        self.settle_time = settle_time

        if set_parameter is None and get_parameter is None:
            raise ValueError(
                "At least one of set_parameter or get_parameter must be provided.")

    def sweep(self, values, measure_func, verbose=True):
        """
        Step through parameter values, running a measurement at each.

        Args:
            values: Iterable of parameter values to sweep through.
            measure_func: Callable(client) -> dict. Called at each parameter
                value to acquire data. Should return a dict of measurement
                results (e.g. from wideband_sweep, get_samples, etc.).
            verbose: Print progress.

        Returns:
            list of MeasurementPoint objects.
        """
        results = []
        values = list(values)
        n = len(values)

        for i, val in enumerate(values):
            if self.set_parameter is not None:
                if verbose:
                    print(f"[{i+1}/{n}] Setting {self.parameter_name} = {val}...")
                self.set_parameter(val)
                if self.settle_time > 0:
                    time.sleep(self.settle_time)

            actual_val = val
            if self.get_parameter is not None:
                actual_val = self.get_parameter()
                if verbose and actual_val != val:
                    print(f"  Readback: {self.parameter_name} = {actual_val}")

            if verbose:
                print(f"  Measuring...")
            data = measure_func(self.client)

            point = MeasurementPoint(
                parameter_value=actual_val,
                parameter_name=self.parameter_name,
                timestamp=time.time(),
                data=data,
                index=i,
            )
            results.append(point)

        if verbose:
            print(f"Sweep complete: {n} points.")
        return results


class TimedMeasurement:
    """
    Take measurements at fixed time intervals.

    Args:
        client: ReadoutClient instance.
        parameter_name: Name of the monitored parameter (e.g. 'temperature').
        get_parameter: Callable() -> value. Reads the parameter at each
            measurement time. Can be None to log only timestamps.
        interval_s: Time between measurements in seconds.
    """

    def __init__(self, client, parameter_name='time',
                 get_parameter=None, interval_s=60.0):
        self.client = client
        self.parameter_name = parameter_name
        self.get_parameter = get_parameter
        self.interval_s = interval_s

    def run(self, measure_func, n_points=None, duration_s=None, verbose=True):
        """
        Run timed measurements.

        Specify either n_points or duration_s. If both are given,
        stops at whichever limit is reached first.

        Args:
            measure_func: Callable(client) -> dict.
            n_points: Number of measurements to take.
            duration_s: Total duration in seconds.
            verbose: Print progress.

        Returns:
            list of MeasurementPoint objects.
        """
        if n_points is None and duration_s is None:
            raise ValueError("Specify n_points or duration_s (or both).")

        results = []
        t_start = time.time()
        i = 0

        while True:
            if n_points is not None and i >= n_points:
                break
            if duration_s is not None and (time.time() - t_start) >= duration_s:
                break

            param_val = None
            if self.get_parameter is not None:
                param_val = self.get_parameter()

            if verbose:
                elapsed = time.time() - t_start
                print(f"[{i+1}] t={elapsed:.1f}s, "
                      f"{self.parameter_name}={param_val}  Measuring...")

            data = measure_func(self.client)

            point = MeasurementPoint(
                parameter_value=param_val,
                parameter_name=self.parameter_name,
                timestamp=time.time(),
                data=data,
                index=i,
            )
            results.append(point)
            i += 1

            # Wait for next interval (accounting for measurement time)
            next_time = t_start + i * self.interval_s
            wait = next_time - time.time()
            if wait > 0:
                time.sleep(wait)

        if verbose:
            print(f"Timed measurement complete: {len(results)} points "
                  f"over {time.time() - t_start:.1f}s.")
        return results


class ConditionalMeasurement:
    """
    Take measurements when an external parameter meets a condition.

    Args:
        client: ReadoutClient instance.
        parameter_name: Name of the monitored parameter.
        get_parameter: Callable() -> value. Reads the parameter.
        condition: Callable(value) -> bool. Returns True when a
            measurement should be taken.
        poll_interval_s: How often to check the condition (seconds).
    """

    def __init__(self, client, parameter_name,
                 get_parameter, condition,
                 poll_interval_s=1.0):
        self.client = client
        self.parameter_name = parameter_name
        self.get_parameter = get_parameter
        self.condition = condition
        self.poll_interval_s = poll_interval_s

    def run(self, measure_func, target_values=None, n_points=None,
            timeout_s=None, verbose=True):
        """
        Poll the parameter and measure when the condition is met.

        Args:
            measure_func: Callable(client) -> dict.
            target_values: List of target parameter values. For each,
                waits until condition(value) is True, then measures.
                Mutually exclusive with n_points.
            n_points: Number of measurements to take (triggers whenever
                condition is met). Mutually exclusive with target_values.
            timeout_s: Maximum time to wait in seconds. None for no timeout.
            verbose: Print progress.

        Returns:
            list of MeasurementPoint objects.
        """
        if target_values is not None and n_points is not None:
            raise ValueError("Specify target_values or n_points, not both.")
        if target_values is None and n_points is None:
            raise ValueError("Specify target_values or n_points.")

        results = []
        t_start = time.time()
        i = 0

        if target_values is not None:
            for target in target_values:
                if verbose:
                    print(f"Waiting for {self.parameter_name} = {target}...")

                while True:
                    if timeout_s and (time.time() - t_start) > timeout_s:
                        if verbose:
                            print("  Timeout reached.")
                        return results

                    val = self.get_parameter()
                    if self.condition(val):
                        if verbose:
                            print(f"  Condition met: {self.parameter_name} = {val}")
                            print(f"  Measuring...")
                        data = measure_func(self.client)
                        results.append(MeasurementPoint(
                            parameter_value=val,
                            parameter_name=self.parameter_name,
                            timestamp=time.time(),
                            data=data,
                            index=i,
                        ))
                        i += 1
                        break
                    time.sleep(self.poll_interval_s)
        else:
            while i < n_points:
                if timeout_s and (time.time() - t_start) > timeout_s:
                    if verbose:
                        print("Timeout reached.")
                    break

                val = self.get_parameter()
                if self.condition(val):
                    if verbose:
                        print(f"[{i+1}/{n_points}] "
                              f"{self.parameter_name} = {val}  Measuring...")
                    data = measure_func(self.client)
                    results.append(MeasurementPoint(
                        parameter_value=val,
                        parameter_name=self.parameter_name,
                        timestamp=time.time(),
                        data=data,
                        index=i,
                    ))
                    i += 1
                time.sleep(self.poll_interval_s)

        if verbose:
            print(f"Conditional measurement complete: {len(results)} points.")
        return results


def save_measurement(results, filepath):
    """
    Save measurement results to a .npz file.

    Args:
        results: List of MeasurementPoint objects.
        filepath: Output path (will add .npz if not present).
    """
    if not filepath.endswith('.npz'):
        filepath += '.npz'

    save_dict = {
        'parameter_name': results[0].parameter_name if results else '',
        'parameter_values': np.array([r.parameter_value for r in results]),
        'timestamps': np.array([r.timestamp for r in results]),
        'n_points': len(results),
    }

    # Store each measurement's data dict entries
    for i, point in enumerate(results):
        for key, value in point.data.items():
            if isinstance(value, np.ndarray):
                save_dict[f'data_{i}_{key}'] = value
            elif isinstance(value, (int, float, np.integer, np.floating)):
                save_dict[f'data_{i}_{key}'] = np.array(value)

    np.savez(filepath, **save_dict)
