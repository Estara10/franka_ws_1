#pragma once

#include <cstddef>

namespace franka_gripper {

/// True only for the explicitly selected finger collision mesh / rod pair.
/// Other callback decisions remain the caller's responsibility.
inline bool isContactProbeMeshRodPair(int geom1, int geom2, int rod_geom,
                                      const int* finger_mesh_geoms,
                                      std::size_t finger_mesh_count) {
  if (rod_geom < 0 || finger_mesh_geoms == nullptr) return false;
  const auto is_finger_mesh = [finger_mesh_geoms, finger_mesh_count](int geom) {
    for (std::size_t i = 0; i < finger_mesh_count; ++i) {
      if (finger_mesh_geoms[i] == geom) return true;
    }
    return false;
  };
  return (geom1 == rod_geom && is_finger_mesh(geom2)) ||
         (geom2 == rod_geom && is_finger_mesh(geom1));
}

/// MuJoCo's built-in geom mask rule (engine_collision_driver.c: filterBitmask).
/// Installing a user contact filter *replaces* this rule for every pair, so the
/// callback must reproduce it explicitly; otherwise geoms disabled via
/// contype=0/conaffinity=0 become collidable again.
inline bool builtinGeomMaskRejects(int contype1, int conaffinity1,
                                   int contype2, int conaffinity2) {
  return !(contype1 & conaffinity2) && !(contype2 & conaffinity1);
}

/// Chain semantics: retain any exclusion made by the previously installed
/// MuJoCo callback, always reproduce the built-in mask rule, then apply the
/// contact-probe-only mesh/rod rule for the active model.
inline bool shouldDiscardContactProbePair(bool previous_callback_discarded,
                                          bool same_model,
                                          int geom1, int contype1, int conaffinity1,
                                          int geom2, int contype2, int conaffinity2,
                                          int rod_geom,
                                          const int* finger_mesh_geoms,
                                          std::size_t finger_mesh_count) {
  if (previous_callback_discarded) return true;
  if (builtinGeomMaskRejects(contype1, conaffinity1, contype2, conaffinity2)) {
    return true;
  }
  return same_model && isContactProbeMeshRodPair(
      geom1, geom2, rod_geom, finger_mesh_geoms, finger_mesh_count);
}

}  // namespace franka_gripper
