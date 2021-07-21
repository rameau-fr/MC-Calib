Contributing
============
All development is done on GitHub: https://github.com/rameau-fr/MultiCamCalib


Git Workflow
============
- ``master`` branch is where all up-to-date stable code is maintained.
- ``feature/feature-name`` is a branch for any improvement, bugs, refactoring or documentation. This branch is the one used to push changes to ``master`` through Pull Requests (PR).

To create a pull request:
=======================

1. Clone the repo.

2. Install `clang-format-lint <https://github.com/DoozyX/clang-format-lint-action>`_:

.. code-block:: bash

    docker build -t clang-format-lint github.com/DoozyX/clang-format-lint-action

3. Make desired changes to the code.

4. Apply clang-format-lint to new changes:

.. code-block:: bash

    docker run -it --rm --workdir /src -v $(pwd):/src clang-format-lint --clang-format-executable /clang-format/clang-format11 -r --inplace True --exclude '.git ./libs' .

5. Make sure new changes pass the tests:

.. code-block:: bash

    ./build/tests/boost_tests_run

6. Create pull request.


Naming convention:
=======================

- variable: the_variable
- member variable: the_variable_
- Our classes/type: CamelCase
- function: camelCase
