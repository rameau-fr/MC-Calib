# Generate documentation

- Doxygen documentation is [available online](https://codedocs.xyz/rameau-fr/MC-Calib/).

- It is also possible to generate Doxygen documentation locally:

   - Install [Doxygen](https://www.doxygen.nl/download.html) or use `bailool/mc-calib-dev` docker image:

      ```bash
      sudo apt install flex
      sudo apt install bison
      git clone https://github.com/doxygen/doxygen.git
      cd doxygen
      mkdir build
      cd build
      cmake -G "Unix Makefiles" ..
      make
      make install # optional
      ```
   - Doxygen is already added to the `CmakeLists.txt` and is auto-generated if dependencies are satisfied. However, it is also possible to set it up manually:

      ```bash
      mkdir docs
      cd docs
      doxygen -g
      #set INPUT = ../src in Doxyfile
      doxygen
      ```