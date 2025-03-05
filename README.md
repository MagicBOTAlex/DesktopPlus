# Desktop+ VR Overlay
Advanced desktop access for OpenVR.

![VR Interface](docs/screenshot.jpg)

## Features

- User interface with real-time adjustments accessible in VR or on desktop
- Smooth, low-latency mirroring of desktops and windows
- Low memory footprint and performance impact
- Support for creating as many overlays as SteamVR allows at once
- Customizable overlay settings (width, position, curvature, opacity, cropping), with switchable profiles
- 3D support (SBS, HSBS, OU, HOU)
- Overlay visibility and origin settings: Display a desktop/window during gameplay or attach it to a different origin (play space, dashboard, HMD, controllers, tracker)
- Actions: User-definable functions (input simulation, running applications) which can be bound to controller inputs, hotkeys or UI buttons
- Custom laser pointer implementation, allowing for non-blocking overlay interaction during gameplay
- Configurable PC-style VR keyboard with various layouts and a keyboard layout editor for deeper customization, appearing automatically when detecting text input widget focus
- Elevated access toggle, making it possible to deal with UAC prompts and other UIP-restricted UI in VR without using full admin-access at all times
- Gaze Fade: Fade-out overlay when not looking at it
- Window management: Change window focus depending on overlay/dashboard state or drag overlays when dragging the title bar of a mirrored window
- Performance Monitor: View system performance in real time
- Application profiles: Automatically switch overlay profiles when a specific VR application is being run
- Browser overlays: View web pages independent from your desktop (CEF-based)

## Usage

### Steam

Install Desktop+ from its [Steam store page](https://store.steampowered.com/app/1494460) and run it.

### Release Archive

Download and extract the latest archive from the [releases page](https://github.com/elvissteinjr/DesktopPlus/releases). Follow instructions in the included [readme file](assets/readme.txt).  
Make sure to also download the [Desktop+ Browser component](https://github.com/elvissteinjr/DesktopPlusBrowser/releases) if you want browser overlay support.

### Nightly Build

An automated build based on the latest code changes can be downloaded from [here](https://nightly.link/elvissteinjr/DesktopPlus/workflows/nightly/master).  
Keep in mind that nightly builds are unstable and mostly untested. Prefer the latest release if possible.

### Building from Source

The Visual Studio 2019 Solution builds out of the box with no further external dependencies.  
Building with Graphics Capture support requires Windows SDK 10.0.19041 or newer, and will download C++/WinRT packages automatically.
Graphics Capture support can be disabled entirely if desired. Windows 8 SDK or newer is sufficient in that case. See DesktopPlusWinRT.h for details.

See the [Desktop+ Browser repository](https://github.com/elvissteinjr/DesktopPlusBrowser) for building the browser component.

Other compilers likely work as well, but are neither tested nor have a build configuration. Building for 32-bit is not supported.

## Demonstration

The [Steam announcements](https://store.steampowered.com/news/app/1494460) for typically feature short video clips showing off new additions.  
The trailer on the [Steam store page](https://store.steampowered.com/app/1494460) also shows off some functionality.

## Documentation

For basic usage, installation and troubleshooting see the included [readme file](assets/readme.txt).  
For more detailed information on each setting, step-by-step examples for a few common usage scenarios and more check out the [User Guide](docs/user_guide.md).

## Notes

Desktop+ only runs on Windows 8.1 or newer, as it uses the DXGI Desktop Duplication API which is not available on older versions of Windows.  
Window mirroring through Graphics Capture requires at least Windows 10 1803 for basic support, Windows 10 2004 or newer for full support (some additional non-essential features require Windows 11 or Windows 11 24H2). 

## License

This software is licensed under the GPL 3.0.  
Desktop+ includes work of third-party projects. For their licenses, see [here](assets/third-party_licenses.txt).
