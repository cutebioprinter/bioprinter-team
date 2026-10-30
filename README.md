# Bioprinter Team

## Project Overview (updated Fall 2026)

This repository contains the software and supporting tools developed for our custom multi-channel bioprinter. The project focuses on developing an integrated system for 3D model slicing, G-code generation, printer control, and synchronized multi-channel material dispensing.

The system uses a Raspberry Pi 5 connected to a Duet 3 Mainboard 6HC for printer communication and control.

## Software Workflow

The bioprinter software follows this general workflow:

1. **3D Model Input:** Import a 3D model for printing.
2. **Custom Slicer:** Process the model and generate printing paths using our custom slicer.
3. **G-code Generation & Processing:** Generate and prepare G-code containing printer movement and material dispensing instructions.
4. **Printer & Pump Control:** Coordinate printer movement and multi-channel pump operation through the Raspberry Pi and Duet 3.
5. **Bioprinting:** Execute synchronized printing and material dispensing.

## Repository Structure

```text
bioprinter-team/
│
├── README.md
├── requirements.txt
│
├── docs/                  # Project documentation
│
├── src/
│   ├── slicer/            # Custom bioprinter slicer
│   ├── gcode/             # G-code generation and processing
│   ├── printer_control/   # Printer communication and pump control
│   ├── hardware_control/  # Hardware and circuit calculations
│   ├── visualization/     # Printing and material visualization
│   └── monitoring/        # Printer monitoring and debugging
│
├── tests/                 # Testing scripts and files
│
├── examples/              # Example models and G-code files
│
└── archive/               # Legacy and backup files
```

## Hardware

- Raspberry Pi 5
- Duet 3 Mainboard 6HC
- Multi-channel pressure pump system

## Getting Started

Clone the repository:

```bash
git clone https://github.com/cutebioprinter/bioprinter-team.git
cd bioprinter-team
```

Install Python dependencies:

```bash
pip install -r requirements.txt
```

Refer to the `docs/` directory for hardware setup, software configuration, and printing instructions.

## Development

This repository is maintained by the bioprinter development team. Contributors should document new functionality, test changes, and use descriptive commit messages.

---

**Bioprinter Team**  
[GitHub Repository](https://github.com/cutebioprinter/bioprinter-team)
