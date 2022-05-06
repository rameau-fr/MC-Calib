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

.. code-block:: bash

    docker run -it --rm --workdir /src -v $(pwd):/src clang-format-lint --clang-format-executable /clang-format/clang-format11 -r --inplace True --exclude '.git ./libs' .

5. Make sure new changes pass the tests. The end-to-end tests rely on `Synthetic Data <https://bosch.frameau.xyz/index.php/s/pLc2T9bApbeLmSz>`_. 
Extract that and place (or symlink) Blender_Images folder under MC-Calib/data/.

.. code-block:: bash

    mkdir build
    cd build
    ./tests/boost_tests_run

6. Run static analysis tools and fix introduced dangerous code constructs:

.. code-block:: bash

    cd build
    apt install cppcheck
    cppcheck ../src

    # known errors:
    logger.h:19:1: error: There is an unknown macro here somewhere. Configuration is required. If BOOST_LOG_GLOBAL_LOGGER is a macro then please configure it. [unknownMacro] BOOST_LOG_GLOBAL_LOGGER(logger, boost::log::sources::severity_logger_mt<boost::log::trivial::severity_level>)

7. Perform valgrind test and fix introduced memory leaks:

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
      ./calibrate ../tests/configs_for_end2end_tests/calib_param_synth_Scenario1.yml

    # current state of this repository:
    ==1616== LEAK SUMMARY:
    ==1616==    definitely lost: 0 bytes in 0 blocks
    ==1616==    indirectly lost: 0 bytes in 0 blocks
    ==1616==      possibly lost: 0 bytes in 0 blocks
    ==1616==    still reachable: 44,906 bytes in 772 blocks
    ==1616==         suppressed: 375,047 bytes in 2,940 blocks

8. Create pull request.


Naming convention:
=======================

- variable: the_variable
- member variable: the_variable\_
- Our classes/type: CamelCase
- function: camelCase
