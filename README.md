## Winwing plugin for X-Plane 12

For X-Plane 12 only.

This plugin allows you to use Winwing devices in X-Plane 12 on MacOS and Linux.
The plugin theoretically works on Windows, but is not required there, as Winwing provides a native Windows driver through the SimAppPro software.

### Installation

1. Download the latest release from the releases page.
2. Unzip the downloaded file.
3. Copy the `Winwing` folder to your X-Plane 12 `Resources/plugins` directory.
4. Start X-Plane 12.
5. Updating can be done by replacing the `Winwing` folder or using Skunkcrafts Updater if you have it installed.

### Usage

- The plugin will automatically detect your Winwing devices.
- There are no user-configurable settings at this time.

### Devices Compatibility

The table below lists the compatibility status of various Winwing devices with the plugin.

| Device Name                   | Compatibility             | Tested | Roadmap / suggestions                               |
| ----------------------------- | ------------------------- | ------ | --------------------------------------------------- |
| URSA MINOR Airline Joystick L | 🟢 Fully implemented      | 🟢     |                                                     |
| URSA MINOR Airline Joystick R | 🟠 Missing USB product ID | 🔴     | If anyone owns this device, please create an issue. |
| MCDU-32                       | 🟢 Fully implemented      | 🟢     |                                                     |
| PFP 3N                        | 🟢 Fully implemented      | 🟢     |                                                     |
| PFP 4                         | 🟢 Fully implemented      | 🟢     |                                                     |
| PFP 7                         | 🟢 Fully implemented      | 🟢     |                                                     |
| Panel of Autopilot 3          | 🔴 Not implemented        | 🔴     |                                                     |
| FCU Unit                      | 🟠 Partly implemented     | 🔴     |                                                     |
| EFIS R                        | 🟠 Partly implemented     | 🔴     |                                                     |
| EFIS L                        | 🟠 Partly implemented     | 🔴     |                                                     |

### Aircraft Compatibility

Below is a table of aircraft that are compatible with the plugin.

| Aircraft             | Compatibility         | Notes                                                                         |
| -------------------- | --------------------- | ----------------------------------------------------------------------------- |
| Toliss A319          | 🟢 Fully implemented  |                                                                               |
| Toliss A320          | 🟢 Fully implemented  |                                                                               |
| Toliss A321          | 🟢 Fully implemented  |                                                                               |
| Laminar A330-300     | 🟢 Fully implemented  |                                                                               |
| AeroGenesis A330-300 | 🟢 Fully implemented  |                                                                               |
| Zibo 737-800         | 🟢 Fully implemented  |                                                                               |
| FlightFactor 777-200 | 🟠 Partly implemented | Need information on how to send FMC button presses.                           |
| SSG 747-800          | 🟠 Partly implemented | Both FMCs need to display the same text in order for their dataref to update. |
| IXEG 737-300         | 🟢 Fully implemented  |                                                                               |

### Known Issues

- It seems that upon startup, the MCDU reverts to the default font, even if you used SimAppPro to change it. This is due to the startup message "waking" up the MCDU, which resets the font. More research is needed on the startup sequence of the MCDU to find a solution.

### Credits

- [@Schenlap](https://github.com/schenlap) for the initial python implementation and providing the HID protocol documentation.
- [@zodiac1214](https://github.com/zodiac1214) for the Ursa Minor Joystick HID protocol.

### Demonstration

<img src="https://github.com/user-attachments/assets/75d4e3e0-af9e-488f-bd5e-2d834bea110d" alt="Airbus A20N" width="256" />
<img src="https://github.com/user-attachments/assets/8f5750e2-f913-479a-9f7a-6e3d6c31382d" alt="Boeing B738" width="256" />
