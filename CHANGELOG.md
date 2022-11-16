# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]
## Removed
- `SerialReader` and the slider box scripts.  Both have been moved to the
  [slider_box](https://github.com/open-dynamic-robot-initiative/slider_box)
  package.


## [2.0.0] - 2021-08-04
## Removed
- Remove master-board/SPI-related modules and the corresponding
  dependencies (#20, #21).

## Fixed
- Make sure CAN frames are initialised to zero (#22).


## [1.0.0] - 2021-07-21
Version 1.0.0


[Unreleased]: https://github.com/open-dynamic-robot-initiative/blmc_drivers/compare/2.0.0...HEAD
[2.0.0]: https://github.com/open-dynamic-robot-initiative/blmc_drivers/compare/1.0.0...2.0.0
[1.0.0]: https://github.com/open-dynamic-robot-initiative/blmc_drivers/releases/tag/v1.0.0
