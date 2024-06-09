Contributing
============
All development is done on GitHub: https://github.com/rameau-fr/MC-Calib


Git Workflow
============
- ``master`` branch is where all up-to-date stable code is maintained.
- ``feature/feature-name`` is a branch for any improvement, bugs, refactoring or documentation. This branch is the one used to push changes to ``master`` through Pull Requests (PR).

To create a pull request:
=========================

1. Clone the repo.

2. Install `clang-format-lint <https://github.com/DoozyX/clang-format-lint-action>`_:

.. code-block:: bash

    docker build -t clang-format-lint github.com/DoozyX/clang-format-lint-action

3. Make desired changes to the code.

4. Apply clang-format-lint to new changes:

for Linux/Unix:

.. code-block:: bash

    docker run -it --rm --workdir /src -v $(pwd):/src clang-format-lint --clang-format-executable /clang-format/clang-format11 -r --inplace True --exclude '.git ./libs' .

for Windows:

.. code-block:: bash

    docker run -it --rm --workdir /src -v ${pwd}:/src clang-format-lint --clang-format-executable /clang-format/clang-format11 -r --inplace True --exclude '.git ./libs' .


5. Now we have to make sure our new changes pass the tests. In order to test them, we must first open docker image 'bailool/mc-calib-prod'. Make sure to change the starter command in your *.sh/*.ps1 file accordingly. The end-to-end tests rely on `Synthetic Data <https://drive.google.com/file/d/1CxaXUbO4E9WmaVrYy5aMeRLKmrFB_ARl/view?usp=sharing>`_. 
Extract that and place (or symlink) Blender_Images folder under MC-Calib/data/.

.. code-block:: bash
                                         
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Debug -DUSE_SANITIZERS=ON ..
    make -j10
    ./tests/boost_tests_run

6. Run static analysis tools and fix introduced dangerous code constructs:

.. code-block:: bash

    cd build
    apt install cppcheck
    cppcheck ../McCalib/src

    # known errors:
    logger.h:19:1: error: There is an unknown macro here somewhere. Configuration is required. If BOOST_LOG_GLOBAL_LOGGER is a macro then please configure it. [unknownMacro] BOOST_LOG_GLOBAL_LOGGER(logger, boost::log::sources::severity_logger_mt<boost::log::trivial::severity_level>)
    ##############

    apt install clang-tidy
    cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug ..
    run-clang-tidy

7. Perform ASanitizer test in the build directory. In order to run the test properly, the synthetic image data or the blender image data must be downloaded and placed in the data folder. 
Refer back to Step 5 for more information. By running calibrate, we will be able to see if the ASanitizer finds any errors or leaks on the way. 

.. code-block:: bash

    ./apps/calibrate/calibrate ../tests/configs_for_end2end_tests/calib_param_synth_Scenario1.yml

8. Perform valgrind test and fix introduced memory leaks:

In order to use Valgrind, ASanitizer must first be disabled. This can be done by recompiling with the ASanitizer option turned off.

.. code-block:: bash

    cd build
    cmake -DCMAKE_BUILD_TYPE=Debug -DUSE_SANITIZERS=OFF ..
    make -j10

Then, run the valgrind test:

.. code-block:: bash

    apt update
    apt install valgrind
    valgrind --leak-check=full \
      --leak-check=full \
      --track-origins=yes \
      --show-reachable=yes \
      --error-limit=no \
      --gen-suppressions=all \
      --verbose \
      --log-file=valgrind-out.txt \
      --suppressions=../tests/valgrind_suppress/opencv_valgrind.supp \
      --suppressions=../tests/valgrind_suppress/opencv_valgrind_3rdparty.supp \
      --suppressions=../tests/valgrind_suppress/boost_valgrind.supp \
      ./apps/calibrate/calibrate ../tests/configs_for_end2end_tests/calib_param_synth_Scenario1.yml

    # current state of this repository:
    ==6274== LEAK SUMMARY:
    ==6274==    definitely lost: 0 bytes in 0 blocks
    ==6274==    indirectly lost: 0 bytes in 0 blocks
    ==6274==      possibly lost: 0 bytes in 0 blocks
    ==6274==    still reachable: 0 bytes in 0 blocks
    ==6274==         suppressed: 420,593 bytes in 3,714 blocks

9. Code coverage testing

.. code-block:: bash

    cd build
    cmake -DCMAKE_BUILD_TYPE=Debug -DENABLE_COVERAGE=true ..
    make -j10
    ./tests/boost_tests_run && make coverage


10. Test python_utils scripts (if applicable)

.. code-block:: bash

    cd python_utils
    ./format.sh
    ./test.sh


11. Create pull request.


Naming convention:
=======================

- variable: the_variable
- member variable: the_variable\_
- Our classes/type: CamelCase
- function: camelCase
