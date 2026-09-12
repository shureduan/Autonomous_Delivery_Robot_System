# Source Selection and Provenance Notes

## Phase 4 selection

The public subset contains twelve Python files and thirteen Unity C# files selected for method inspection. It covers Unity sensing and scene export, Python occupancy mapping, multi-floor topology construction, Theta* planning, task-distance generation, multi-trip multi-commodity CVRP optimization, waypoint expansion, Unity route execution, Pure Pursuit, elevator interaction, randomized delivery targets, and the Python PARS research environment.

The project owner confirms that the Independent Study's research design, Python implementation, Unity implementation, experiment design, and manuscript were independently completed. File-level review found no third-party copyright header or external-source attribution in the included code. The first-release selection retains only the twelve Python and thirteen C# files summarized here; internal file-by-file review materials are not part of the release.

## Exclusions

The selection excludes old and blocked-path variants, debug/test scripts, plotting utilities, local file watchers, checkpoint sweep tools, version-specific training code, raw data, maps, outputs, logs, checkpoints, environments, caches, and the full Unity project. It also excludes Unity scenes, packages, prefabs, models, textures, audio, `.meta` files, and editor/sample/Asset Store scripts.

`LightmappedLOD.cs` was identified as dated, unrelated Asset Store code and excluded. The old `TestAgent.cs` ML-Agents prototype was excluded because it is not part of the final method and its generic sample derivation could not be ruled out.

## Sanitization

Public copies use repository-relative paths or caller-supplied command-line inputs. No original research file was modified. Historical datasets and exact dependency versions were not reconstructed or inferred.

## Runtime boundary

This is a minimal research-code release, not an end-to-end runnable package. The Python PARS environment contains switching and path-rejoin behavior, but no live Unity SAC-inference/control-return bridge was located in the reviewed source. The release documents that boundary without claiming that such a bridge never existed elsewhere historically.

No open-source license is granted for this release.
