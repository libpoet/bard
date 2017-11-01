# Release Notes

## [Unreleased]


## [bard/v2.0.0] - 2017-11-01
### Added
 * SAMOS 2016 publication details
 * RELEASES-bard.md to track Bard-specific changes (not those merged from upstream POET)

### Changed
 * Merge from upstream [POET v2.0.1](https://github.com/libpoet/poet/releases/tag/v2.0.1) ([POET changes since last merge](https://github.com/libpoet/poet/compare/3b32625...v2.0.1); see RELEASES.md for summary); highlights include:
  * Switch to CMake build system
  * Remove dependency on heartbeats; add optional test dependencies on heartbeats-simple and energymon
  * Lots of fixes based on static analysis and testing with stricter compile flags
 * Renamed library from poet to bard so as not to conflict with POET installations
 * Rename poet_idle to bard_idle
 * No longer hardcoding location of bard_idle application, should be on PATH
 * Major version bump due to backward-incompatible changes


## [bard/v1.0.0] - 2016-05-13
### Added
 * Initial public release (forked Bard from POET git revision 3b32625)

[Unreleased]: https://github.com/libpoet/bard/compare/bard/v2.0.0...HEAD
[bard/v2.0.0]: https://github.com/libpoet/bard/compare/bard/v1.0.0...bard/v2.0.0
[bard/v1.0.0]: https://github.com/libpoet/bard/compare/3b32625...bard/v1.0.0
