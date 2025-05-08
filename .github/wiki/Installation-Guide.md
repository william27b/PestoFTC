# Installation Guide

This guide will walk you through the process of installing PestoFTC in your FTC project.

## Prerequisites
- Android Studio
- FTC SDK project
- Basic understanding of Gradle build system

## Installation Steps

1. **Download the Library**
   - Download the latest `.aar` file from the [releases page](https://github.com/william27b/PestoFTC/releases)
   - Alternatively, you can find it in the main branch at `PestoCore/build/outputs/aar/PestoCore-release.aar`

2. **Add to Your Project**
   - Create a `libs` folder in your FTC project if it doesn't exist
   - Copy the downloaded `.aar` file into the `libs` folder

3. **Configure Gradle**
   - Open your TeamCode module's `build.gradle` file
   - Add the following line inside the `dependencies` block:
   ```gradle
   implementation files('../libs/PestoCore-release.aar')
   ```

4. **Sync Project**
   - Click "Sync Now" in Android Studio
   - Wait for the Gradle sync to complete

## Verification
To verify the installation:
1. Open your project
2. Try importing a PestoFTC class
3. If no errors appear, the installation was successful

## Troubleshooting
If you encounter any issues:
- Ensure the `.aar` file is in the correct location
- Check that the Gradle dependency is properly added
- Verify that you're using a compatible version of the FTC SDK
- Check the [GitHub issues](https://github.com/william27b/PestoFTC/issues) for known problems 