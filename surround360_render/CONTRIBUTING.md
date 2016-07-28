# Contributing to Surround 360

We want to make contributing to this project as easy and transparent as
possible.

## Our Development Process

We maintain a repository for Surround 360 using Facebook's internal infrastructure, which is automatically synched with GitHub.

## Pull Requests

We actively welcome your pull requests.

1. Fork the repo and create your branch from `master`.
2. If you've added code that should be tested, add tests.
3. If you've changed APIs, update the documentation.
4. Ensure the test suite passes.
5. Make sure your code lints.
6. If you haven't already, complete the Contributor License Agreement ("CLA").

## Contributor License Agreement ("CLA")

In order to accept your pull request, we need you to submit a CLA. You only need
to do this once to work on any of Facebook's open source projects.

Complete your CLA here: <https://code.facebook.com/cla>

## Issues

We use GitHub issues to track public bugs. Please ensure your description is
clear and has sufficient instructions to be able to reproduce the issue.

Facebook has a [bounty program](https://www.facebook.com/whitehat/) for the safe
disclosure of security bugs. In those cases, please go through the process
outlined on that page and do not file a public issue.

## Coding Style

* 2 spaces for indentation rather than tabs
* 80 character line length (with discretion in cases where it makes code less readable).
* use `#pragma` once instead of `#ifdef` include guards
* Indentation style for functions should be one of these:

  If the whole thing fits on one line:

  ```
  void foo(int x, int y) {
    body
  }
  ```

  If it does not fit on one line:

  ```
  void foo(
      int x,
      int y) {

    body
  }
  ```

  Note that in the above example the argument list is indented 2 levels relative to the function name. This should be used for implementations. For headers/forward declarations, only indent 1 level.

  Avoid this style:

  ```
  void bazz(int x,
            int y) {
  }
  ```

  Also note that we follow similar conventions for function calls. The following patterns should be preferred:

  If it fits all on one line:

  ```
  int x = foo(a, b, c);
  ```

  Two lines:

  ```
  int x = foobarscrazzle(
    x, y, z, a, b, c);
  ```

  Many lines:

  ```
  int x = foobarscrazzle(
    foo,
    bar,
    blah);
  ```

  Avoid this:

  ```
  int x = foobarscrazzled(xyz,
                          abc);
  ```

* Group asterisk and ampersand with the type instead of the variable name, e.g., this `int* x;` vs. `int *x;`


## License

The Surround 360 rendering code is BSD-licensed, as it appears in `LICENSE_render.md` under `/surround360_render`. We also provide an additional patent grant.
