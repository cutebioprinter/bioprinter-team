# Custom Bioprinter Slicer

## Overview

This directory contains the development of our custom slicer for the multi-channel bioprinter.

The slicer is responsible for converting 3D models into printing toolpaths and generating G-code tailored to our printer's hardware and material-dispensing requirements.

Our long-term goal is to develop an independent slicing workflow rather than rely on third-party slicers.

## Current Development

The slicer is currently being developed in Google Colab using a Jupyter notebook.

- `bioprinter_slicer.ipynb` — Main development notebook for the custom slicer.

To open the notebook directly in Google Colab, use:

[Open in Google Colab](https://colab.research.google.com/github/cutebioprinter/bioprinter-team/blob/main/src/slicer/bioprinter_slicer.ipynb)


## Development Workflow

1. Develop and test slicer functionality in Google Colab.
2. Save the notebook and commit updated versions to GitHub.
3. Test generated G-code with the printer-control and pump-control software.
4. Document changes to the slicing workflow and G-code format.

As the slicer matures, functionality may be separated into Python modules to make the software easier to maintain and integrate with the rest of the system.


