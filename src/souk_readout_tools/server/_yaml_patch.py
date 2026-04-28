"""
Legacy line-based YAML patcher.

This helper is not used by the readout server anymore.  `pull_config()` now
returns the active config file exactly as stored, and runtime hardware state is
reported through status/info calls or explicitly captured by the client sync
methods.
"""

import re
import yaml

# ``key: value`` capture.  The key is the bareword on the left, the rest
# of the line goes through the value/trail split below.
_KV_RE = re.compile(r'^(?P<indent>[ \t]*)(?P<key>[A-Za-z0-9_\-]+):(?P<rest>.*)$')


def _split_value_and_trail(rest):
    """Split the post-colon portion into (sep, value, trail-comment).

    ``sep`` is the whitespace immediately after the colon; ``trail`` is
    the trailing inline comment (including its leading whitespace and
    ``#``).  ``value`` is whatever sits between them.

    Quoted strings in the value can legally contain ``#``; we honour
    quoting so an inline comment isn't accidentally chopped off mid-string.
    """
    sep_match = re.match(r'^([ \t]*)', rest)
    sep = sep_match.group(1)
    body = rest[len(sep):]

    in_squote = in_dquote = False
    i = 0
    while i < len(body):
        c = body[i]
        if c == "'" and not in_dquote:
            in_squote = not in_squote
        elif c == '"' and not in_squote:
            in_dquote = not in_dquote
        elif c == '#' and not in_squote and not in_dquote:
            # Comment must be preceded by whitespace to count as inline
            if i == 0 or body[i - 1] in ' \t':
                break
        i += 1

    value = body[:i].rstrip()
    trail_start = body[i:]
    if trail_start:
        # Push any trailing whitespace from value into the gap before the comment
        gap_len = len(body[:i]) - len(value)
        trail = ' ' * gap_len + trail_start if gap_len else trail_start
    else:
        trail = ''
    return sep, value, trail


def _format_scalar(v):
    """Format a Python scalar as YAML.  None → empty (key holds no value)."""
    if v is None:
        return None
    if isinstance(v, bool):
        return 'true' if v else 'false'
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        # repr keeps full precision; yaml.safe_load round-trips it
        return repr(v)
    if isinstance(v, str):
        # Quote when ambiguous with another type or contains structural chars
        needs_quote = (
            not v
            or v[0] in ' \t' or v[-1] in ' \t'
            or v.lower() in ('true', 'false', 'null', 'yes', 'no', 'on', 'off', '~')
            or any(c in v for c in ':#[]{},&*!|>%@`')
        )
        if not needs_quote:
            try:
                round_trip_ok = yaml.safe_load(v) == v
            except Exception:
                round_trip_ok = False
            if not round_trip_ok:
                needs_quote = True
        if needs_quote:
            escaped = v.replace('\\', '\\\\').replace('"', '\\"')
            return f'"{escaped}"'
        return v
    # Fallback: not a supported scalar — caller should skip these
    return None


def patch_yaml_text(raw_text, config):
    """Return ``raw_text`` with scalar leaves updated from ``config``.

    Walks the text line-by-line, tracks the active key path by indent,
    and only rewrites a line when the corresponding ``config`` leaf is a
    scalar that differs from the parsed YAML value.  Lines that don't
    match a simple ``key: value`` pattern, or whose leaf in ``config`` is
    a mapping/sequence/missing, pass through unchanged — comments and
    layout stay exactly as the user wrote them.
    """
    out = []
    stack = []  # (indent, key) pairs forming the current path

    for line in raw_text.split('\n'):
        m = _KV_RE.match(line)
        if not m:
            out.append(line)
            continue

        indent = len(m.group('indent'))
        key = m.group('key')
        sep, value_str, trail = _split_value_and_trail(m.group('rest'))

        while stack and stack[-1][0] >= indent:
            stack.pop()
        path = [k for _, k in stack] + [key]

        leaf = config
        try:
            for k in path:
                leaf = leaf[k]
            found = True
        except (KeyError, TypeError):
            leaf = None
            found = False

        # Empty value → either a mapping start or an empty scalar.
        if not value_str:
            if isinstance(leaf, dict):
                stack.append((indent, key))
                out.append(line)
                continue
            # Empty scalar — fill in if the dict now holds a real scalar
            if found and leaf is not None and not isinstance(leaf, (dict, list)):
                new_str = _format_scalar(leaf)
                if new_str is not None:
                    # If a trailing comment exists, make sure there's whitespace
                    # before the ``#`` so it's still recognised as inline.
                    safe_trail = trail
                    if safe_trail and not safe_trail[0].isspace():
                        safe_trail = '  ' + safe_trail
                    out.append(f"{m.group('indent')}{key}: {new_str}{safe_trail}")
                    continue
            out.append(line)
            continue

        # Has a value — patch only if the dict has a differing scalar
        if not found or leaf is None or isinstance(leaf, (dict, list)):
            out.append(line)
            continue

        try:
            parsed = yaml.safe_load(value_str)
        except Exception:
            parsed = value_str

        if parsed == leaf:
            out.append(line)
            continue

        new_str = _format_scalar(leaf)
        if new_str is None:
            out.append(line)
            continue

        sep_out = sep if sep else ' '
        out.append(f"{m.group('indent')}{key}:{sep_out}{new_str}{trail}")

    return '\n'.join(out)
