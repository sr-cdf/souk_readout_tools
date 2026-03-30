"""
Shared configuration utilities for souk_readout_tools.

Functions for locating and copying template config files, used by both
the client and server packages.
"""

import os
import re
from datetime import date
from importlib.resources import files as importlib_files


def get_template_config_path():
    """Return the path to the bundled template config file in the package data."""
    return str(importlib_files('souk_readout_tools').joinpath('data', 'config', 'template_config.yaml'))


def copy_template_config(destination, pipeline_id=0, config_id=None,
                         created_by=None, comments=None):
    """
    Copy the template config to a destination file, updating pipeline-specific fields.

    Copies the template as raw text so that comments are preserved.

    Args:
        destination: Path to write the config file.
        pipeline_id: Pipeline ID (0 or 1) to set in the template.
        config_id: Optional config identifier string.
        created_by: Optional author/creator string.
        comments: Optional comments string.
    """
    template_src = get_template_config_path()

    with open(template_src, 'r') as f:
        text = f.read()

    # Update creation date to today
    text = re.sub(
        r'(creation_date:\s*)"[^"]*"',
        rf'\1"{date.today().isoformat()}"',
        text,
    )

    # Optional metadata overrides
    if config_id is not None:
        text = re.sub(
            r'(config_id:\s*)"[^"]*"',
            rf'\1"{config_id}"',
            text,
        )
    if created_by is not None:
        text = re.sub(
            r'(created_by:\s*)"[^"]*"',
            rf'\1"{created_by}"',
            text,
        )
    if comments is not None:
        text = re.sub(
            r'(  comments:\s*)"[^"]*"',
            rf'\1"{comments}"',
            text,
            count=1,
        )

    # Pipeline-specific fields
    text = re.sub(
        r'(pipeline_id:\s*)0',
        rf'\g<1>{pipeline_id}',
        text,
    )
    text = re.sub(
        r'(request_port:\s*)10000',
        rf'\g<1>{10000 + pipeline_id}',
        text,
    )
    text = re.sub(
        r'(stream_port:\s*)20000',
        rf'\g<1>{20000 + pipeline_id}',
        text,
    )

    # RFDC tile/block mapping for pipeline 1
    if pipeline_id == 1:
        text = re.sub(r'(dac0_tile:\s*)0', r'\g<1>1', text)
        text = re.sub(r'(dac1_tile:\s*)0', r'\g<1>1', text)
        text = re.sub(r'(adc_tile:\s*)2', r'\g<1>3', text)

    with open(destination, 'w') as f:
        f.write(text)

    print(f'Template config written to {destination}')
    print(f'Edit this file with your system-specific settings before use.')
