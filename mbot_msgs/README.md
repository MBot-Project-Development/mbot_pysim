This is copy pasted from mbot_lcm_base. The serialize part is removed from CMakelists.txt, and the CMakelists.txt is modified to use python virtual machine only.

## Installation

To compile and install these messages from source, in the root of this repo, activate your virtual environment,

then:
```bash
mkdir build
cd build
cmake ..
make
make install
```
To uninstall, in the build folder, do: `xargs rm < install_manifest.txt`
