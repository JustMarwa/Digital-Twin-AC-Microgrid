# Digital Twin for an AC Microgrid

EEE4022S final-year project (2026), University of Cape Town.

This repository holds the Simulink models, data-acquisition software, PLC program and results for a MATLAB/Simulink Digital Twin of the UCT laboratory AC microgrid rig. Section numbers below refer to the project report.

![PLC-based contactor switching on the Digital Twin dashboard](Implemeted%20PLC%20Based%20Contactor%20Switching%20on%20DT%20Dashboard.png)

## Repository contents

### Digital Twin and data acquisition

| File | Report section | Description |
| --- | --- | --- |
| [`DAQ_with_EMS_Dashboard_v01.slx`](DAQ_with_EMS_Dashboard_v01.slx) | 3.2 – 3.5, 4.4 | **Main Digital Twin prototype.** Contains every Digital Twin subsystem except the offline models: (1) the DAQ pipeline, (2) bidirectional PLC contactor switching from the Digital Twin, and (3) all visualisation dashboard components, i.e. virtual meter viewing (3.5.1), inter-bus voltage angle viewing (3.5.2) and system loading viewing (3.5.3). |
| [`modbus_meter_dashboard_Delay_Resolution.py`](modbus_meter_dashboard_Delay_Resolution.py) | 3.2 | Python DAQ application. Owns a single Modbus TCP client connected to the rig's existing HMI gateway and reads all 11 DIRIS meters. |
| [`plc_contactor.py`](plc_contactor.py) | 3.4, 4.4 | Python PLC contactor controller and Simulink bridge. Replaces WPLSoft's "Set On / Set Off" by sending the same Modbus ASCII Force Single Coil (FC05) messages to the rig's Delta DVP PLC over RS-232 (COM7, 9600 baud, 7E1, station 0). Its `bridge` mode makes the Digital Twin bidirectional: switch commands from the Simulink dashboard arrive over a virtual serial pair (COM6 → COM5) and are applied to the PLC, while the live contactor states (Y outputs) and the X0 master-enable bit are sent back to Simulink. Maps 18 contactors across the GENCOR, POOLCOR and CUSCOR buses. Built-in safety: synchronising switches SG22, SG44 and SP1 can only be opened by software (never closed), X0 is checked before any close, every command asks for confirmation from the command line, and a `DRY_RUN` mode prints messages without sending them. Every request and PLC readback is time-stamped in `actuation_log.csv`. Commands: `frame`, `read`, `status`, `watch`, `on`, `off`, `listen`, `bridge` (run with no arguments for help). |
| [`Dvp0.dvp`](Dvp0.dvp) | 3.4.1 | Original UCT rig PLC program (Delta WPLSoft). |
| [`Implemeted PLC Based Contactor Switching on DT Dashboard.png`](Implemeted%20PLC%20Based%20Contactor%20Switching%20on%20DT%20Dashboard.png) | 3.4, 4.4 | Bidirectional PLC contactor switching implemented on the MATLAB Digital Twin dashboard. |

### Offline models

| File | Report section | Description |
| --- | --- | --- |
| [`Rev_0_UCT_Microgrid_Microgrid.slx`](Rev_0_UCT_Microgrid_Microgrid.slx) | 3.1 | Revision 0 offline model of the UCT microgrid. |
| [`Case_IS_1.slx`](Case_IS_1.slx) | 4.2.1 | Islanded Case IS-1 model. |
| [`Case_IS_2_Model.slx`](Case_IS_2_Model.slx) | 4.2.2 | Islanded Case IS-2 model. |
| [`Case_GC_1_Model.slx`](Case_GC_1_Model.slx) | 4.3.1 | Grid-connected Case GC-1 model. |

### Figures and results

| Folder / file | Description |
| --- | --- |
| [`Methodology/`](Methodology) | Methodology figures: DAQ pipeline, offline model single-line diagram, Revision 0 cycle 1 model, SYNC 2 synchronisation scheme. |
| [`Results_new/`](Results_new) | Results figures: lab vs Simulink comparisons for Cases IS-1, IS-2 and GC-1 (single-line diagrams, physical rig photos, line-to-line voltages, frequency/voltage drop), virtual DIRIS meter matching, dashboard layouts (system loading, inter-bus angle, diagnostics) and live visualisations. |
| [`Case GC-1 for Appendix.pdf`](Case%20GC-1%20for%20Appendix.pdf) | Case GC-1 supporting material for the report appendix. |

## Requirements

- **MATLAB / Simulink R2024b** (all `.slx` models were saved in R2024b), with:
  - **Simscape Electrical** (Specialized Power Systems) for the offline power-system models;
  - **Instrument Control Toolbox** for the serial communication blocks in the Digital Twin dashboard model.
- **Python 3** for the DAQ application and PLC contactor controller, with `pymodbus` and `pyserial` (`tkinter` ships with standard Python):

  ```bash
  pip install pymodbus pyserial
  ```

  Set `HMI_IP`, `HMI_PORT` and `UNIT_ID` at the top of the DAQ script to match your HMI gateway before running.

  For `plc_contactor.py`, close WPLSoft first (only one program can use the PLC COM port), check `PLC_PORT` and `BRIDGE_PORT` at the top of the script, and leave `DRY_RUN = True` until the messages have been checked. Example:

  ```bash
  python plc_contactor.py status
  ```
- **Delta WPLSoft** to open `Dvp0.dvp`.

## Note on large files

Simulink models (`.slx`) are stored with [Git LFS](https://git-lfs.com/). Install Git LFS before cloning so the models download correctly:

```bash
git lfs install
```

```bash
git clone https://github.com/JustMarwa/Digital-Twin-AC-Microgrid.git
```

Downloading the repository as a ZIP from GitHub also includes the LFS files.
