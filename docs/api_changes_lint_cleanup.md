# ED public API changes (clang-tidy cleanup)

> **Who needs this:** anyone maintaining a package that includes `ed/*.h` or
> derives from `ed::Plugin` — `ed_localization`, `ed_gui_server`,
> `ed_sensor_integration`, `ed_navigation`, `ed_perception`, `ed_moveit`,
> `ed_rviz_plugins`, `ed_tutorials`.
>
> These changes landed while bringing `ed` to zero clang-tidy findings on the
> ROS 2 branch. They are **source-breaking**: a dependent package will not
> compile until it is updated. None of them change runtime behaviour.

---

## 0. Find your call sites first

Run these from the root of your package. Everything they report needs an edit.

```bash
# renamed member functions
grep -rn --include=*.h --include=*.cpp -E \
  '\bhas_pose\s*\(|\bworld_model\s*\(|\bentity_(revisions|shape_revisions|visual_revisions|collision_revisions|volumes_revisions)\s*\(' .

# enumerators that moved into a scope
grep -rn --include=*.h --include=*.cpp -E \
  '\b(NoVolumes|ModelVolumes|RoomVolumes)\b|\bShowVolumes\b' .
grep -rn --include=*.h --include=*.cpp -E \
  '\bed::io::(ARRAY|MAP|VALUE)\b|NodeType' .

# types whose constructors became explicit
grep -rn --include=*.h --include=*.cpp -E \
  '\b(ed::Entity|ed::WorldModel|ed::ErrorContext|ed::EventClock|ed::LoopUsageStatus)\b|\bio::(JSONReader|JSONWriter|Writer|DataWriter)\b' .

# integer types in signatures you override or store
grep -rn --include=*.h --include=*.cpp -E '\bunsigned long\b' .

# plugins that no longer exist
grep -rn -E 'libed_gui_plugin|libed_builder_plugin|ed/gui\b|ed/builder\b' .
```

---

## 1. Renamed member functions

`readability-identifier-naming` requires `camelBack` for functions. Pure
renames — same return type, same arguments, same semantics.

| Was | Now | Declared in |
| --- | --- | --- |
| `Entity::has_pose()` | `Entity::hasPose()` | `ed/entity.h` |
| `Server::world_model()` | `Server::worldModel()` | `ed/server.h` |
| `WorldModel::entity_revisions()` | `WorldModel::entityRevisions()` | `ed/world_model.h` |
| `WorldModel::entity_shape_revisions()` | `WorldModel::entityShapeRevisions()` | `ed/world_model.h` |
| `WorldModel::entity_visual_revisions()` | `WorldModel::entityVisualRevisions()` | `ed/world_model.h` |
| `WorldModel::entity_collision_revisions()` | `WorldModel::entityCollisionRevisions()` | `ed/world_model.h` |
| `WorldModel::entity_volumes_revisions()` | `WorldModel::entityVolumesRevisions()` | `ed/world_model.h` |

`entityShapeRevisions()` was already `[[deprecated]]` before this change and
still is — it forwards to `entityVisualRevisions()`. If you are touching these
call sites anyway, move off it now.

A sed that covers all seven:

```bash
grep -rlZ --include=*.h --include=*.cpp -E \
  'has_pose\(|world_model\(|entity_(revisions|shape_revisions|visual_revisions|collision_revisions|volumes_revisions)\(' . \
| xargs -0 sed -i \
  -e 's/\bhas_pose(/hasPose(/g' \
  -e 's/\bworld_model(/worldModel(/g' \
  -e 's/\bentity_revisions(/entityRevisions(/g' \
  -e 's/\bentity_shape_revisions(/entityShapeRevisions(/g' \
  -e 's/\bentity_visual_revisions(/entityVisualRevisions(/g' \
  -e 's/\bentity_collision_revisions(/entityCollisionRevisions(/g' \
  -e 's/\bentity_volumes_revisions(/entityVolumesRevisions(/g'
```

> **Quirk:** `ed_interfaces/msg/EntityInfo` has a **field** called `has_pose`.
> The sed above only matches `has_pose(` with the opening parenthesis, so
> `msg.has_pose = e.hasPose();` comes out right. Do not blanket-rename
> `has_pose` without the parenthesis.

---

## 2. Enums are now scoped

### `ed::ShowVolumes` — `ed/rendering.h`

```cpp
// was
enum ShowVolumes { NoVolumes, ModelVolumes, RoomVolumes };

// now
enum class ShowVolumes : std::uint8_t { NO_VOLUMES, MODEL_VOLUMES, ROOM_VOLUMES };
```

| Was | Now |
| --- | --- |
| `ed::NoVolumes` | `ed::ShowVolumes::NO_VOLUMES` |
| `ed::ModelVolumes` | `ed::ShowVolumes::MODEL_VOLUMES` |
| `ed::RoomVolumes` | `ed::ShowVolumes::ROOM_VOLUMES` |

The enumerator values (0, 1, 2) are unchanged, but a scoped enum no longer
converts to an integer on its own. Arithmetic on it needs an explicit cast:

```cpp
// was
show_volumes = ed::ShowVolumes((show_volumes + 1) % 3);

// now
show_volumes = static_cast<ed::ShowVolumes>((static_cast<int>(show_volumes) + 1) % 3);
```

`renderWorldModel()` also lost the redundant `const` on its `show_volumes`
parameter. That is not source-breaking — top-level `const` on a by-value
parameter was never part of the signature.

### `ed::io::NodeType` — `ed/io/data.h`

```cpp
// was
enum NodeType { ARRAY, MAP, VALUE };

// now
enum class NodeType : std::uint8_t { ARRAY, MAP, VALUE };
```

Qualify the enumerators: `ed::io::ARRAY` → `ed::io::NodeType::ARRAY`, and the
same for `MAP` and `VALUE`. This only matters if you construct `ed::io::Node`
or inspect `Node::type` directly.

### `ed::models::LoadType` — `ed/models/model_loader.h`

Already a scoped enum; it only gained a `std::uint8_t` base type. **No source
change needed** — but note the size changed from 4 to 1 byte, so anything that
serialises it raw needs recompiling (as does everything else here).

---

## 3. Constructors that became `explicit`

Implicit conversion into these types no longer compiles. Spell the type out.

| Type | Header |
| --- | --- |
| `ed::Entity` | `ed/entity.h` |
| `ed::WorldModel` | `ed/world_model.h` |
| `ed::WorldModel::EntityIterator` | `ed/world_model.h` |
| `ed::ErrorContext` | `ed/error_context.h` |
| `ed::EventClock` | `ed/event_clock.h` |
| `ed::LoopUsageStatus` | `ed/loop_usage_status.h` |
| `ed::io::JSONReader` | `ed/io/json_reader.h` |
| `ed::io::JSONWriter` | `ed/io/json_writer.h` |
| `ed::io::Writer` | `ed/io/writer.h` |
| `ed::io::DataWriter` | `ed/io/data_writer.h` |

```cpp
// was
ed::io::JSONReader r = req.request.c_str();
return {entities_};                       // returning an EntityIterator

// now
ed::io::JSONReader r(req.request.c_str());
return const_iterator{entities_};         // direct-init still works with explicit
```

**Deliberately left implicit** — do not "fix" these, they carry a `NOLINT` with
the reason:

| Type | Why |
| --- | --- |
| `ed::UUID` | A transparent string wrapper; `ed::UUID id = "foo";` is the intended spelling |
| `ed::Time` | A transparent seconds wrapper; `ed::Time t = 1.5;` is intended |
| `ed::Variant` (`ed/variant.h`) | A variant is built from its alternatives |
| `ed::io::Variant` (`ed/io/variant.h`) | Same |

---

## 4. Integer types

`google-runtime-int` wants fixed-width types in interfaces. Every `unsigned
long` in a public signature became `std::uint64_t`, and the POSIX `uint` in
`ed/models/shape_loader.h` became `std::uint32_t`.

| Header | Affected |
| --- | --- |
| `ed/entity.h` | `shapeRevision()`, `visualRevision()`, `collisionRevision()`, `volumesRevision()`, `Property::revision()`, `setRevision()` |
| `ed/world_model.h` | `revision()` and all five `entity*Revisions()` accessors |
| `ed/property.h` | `PropertyKeyDBEntry::revision` |
| `ed/models/shape_loader.h` | `getMiddlePoint()`, `createSphere()` |

**On Linux `std::uint64_t` *is* `unsigned long`**, so ordinary calls,
assignments and references keep compiling untouched. Two things do need
attention:

- Exact template or overload matches on `const std::vector<unsigned long>&`
  keep working for the same reason, but write `std::uint64_t` in new code.
- If you build for a platform where `uint64_t` is `unsigned long long`
  (some non-Linux targets), a `const std::vector<unsigned long>&` will no
  longer bind. Use `const auto&` or `std::uint64_t` at those call sites.

Add `#include <cstdint>` where you now name these types.

---

## 5. Changed signatures

| Was | Now | Where |
| --- | --- | --- |
| `Probe::srvCallback(std::shared_ptr<...Request> ros_req, ...)` | `Probe::srvCallback(const std::shared_ptr<...Request>& ros_req, ...)` | `ed/io/transport/probe.h` |
| `getMiddlePoint(geo::Mesh&, uint, uint, std::map<unsigned long, uint>, double)` | `getMiddlePoint(geo::Mesh&, std::uint32_t, std::uint32_t, std::map<std::uint64_t, std::uint32_t>, double)` | `ed/models/shape_loader.h` |
| `createSphere(geo::Shape&, double, uint)` | `createSphere(geo::Shape&, double, std::uint32_t)` | `ed/models/shape_loader.h` |

If you subclass `ed::io::Probe`, your `srvCallback` override must take the
request by const reference or it will silently stop overriding. Add `override`
to catch that at compile time.

`Plugin::configure(tue::Configuration config)` **keeps taking the config by
value.** clang-tidy asks for a const reference; that would break every
override, because `tue::Configuration` is a `ReaderWriter` handle and
`readGroup()`/`readArray()` mutate its cursor. Each plugin needs its own
cursor. This is documented in-place with a `NOLINT`.

---

## 6. Removed

- **`ed/gui` and `ed/builder` plugins.** `plugins/gui_plugin.*` and
  `plugins/builder_plugin.*` were deleted. Neither was listed in
  `CMakeLists.txt` or `plugins.xml` on the `ros1` branch either, so they have
  not been built for years. If a config still names `libed_gui_plugin.so` or
  `libed_builder_plugin.so`, drop the entry — the GUI role belongs to
  `ed_gui_server`.
- **`LoopUsageStatus::start_` / `::duration_`** — private, unused, never
  readable from outside. No downstream impact.

---

## 7. Names that stayed snake_case on purpose

These carry a `NOLINT` and should not be renamed:

| Name | Why |
| --- | --- |
| `ed::UUID::c_str()` | Mirrors `std::string::c_str()` |
| `ed::ImageMask::const_iterator` | Mirrors the standard container spelling; renaming breaks range-for and iterator traits |
| The `MyHandler` members in `src/io/json_reader.cpp` | Dictated by rapidjson's SAX Handler concept |

---

## 8. Build-side change: `libomp-21-dev`

`ed`'s `package.xml` gained `<test_depend>libomp-21-dev</test_depend>`.

38 of `ed`'s translation units compile with `-fopenmp`, inherited from PCL's
imported target. clang-tidy runs a *clang* frontend, which then needs its own
`omp.h`; without it Eigen fails to include it, every affected translation unit
aborts with `Error while processing …`, and what analysis survives reports
findings that are simply untrue (a `const double x = v[i];` reported as
uninitialised, a variable passed to a non-const reference reported as
const-able).

**If your package also links PCL and runs `ament_clang_tidy`, add the same test
dependency** — otherwise your lint results are not trustworthy either. Check
with:

```bash
grep -c -- -fopenmp build/<pkg>/compile_commands.json
clang-tidy-21 -p build/<pkg> <a source file> 2>&1 | grep 'omp.h'
```

---

## 9. Validate

```bash
colcon build --packages-up-to <your_package>
colcon test --packages-select <your_package> && colcon test-result --verbose

# nothing left from the old API:
grep -rn --include=*.h --include=*.cpp -E \
  '\bhas_pose\s*\(|\bworld_model\s*\(|\bentity_[a-z_]*revisions\s*\(|\b(NoVolumes|ModelVolumes|RoomVolumes)\b' .
```

Checklist:

- [ ] All seven renamed accessors updated; `msg.has_pose` **field** untouched.
- [ ] `ShowVolumes` enumerators scoped; any arithmetic on them casts explicitly.
- [ ] `ed::io::NodeType` enumerators qualified.
- [ ] No implicit conversions into the now-`explicit` types.
- [ ] `#include <cstdint>` added where `std::uint64_t`/`std::uint32_t` is named.
- [ ] `Probe::srvCallback` overrides take the request by const reference and are
      marked `override`.
- [ ] `libed_gui_plugin.so` / `libed_builder_plugin.so` gone from configs.
- [ ] `libomp-21-dev` added as a test dependency if the package links PCL.
