#pragma once

#include "collision_element_impl.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace collision {

CollisionElementImpl::CollisionElementImpl(Eigen::Isometry3d& Teg): Teg_(Teg) {
  PRINT_VAR(__PRETTY_FUNCTION__);
}

void CollisionElementImpl::update_geometry_to_element_transform(
    const Eigen::Isometry3d &T_EG){
  if(bullet_pimpl_)
    bullet_pimpl_->update_geometry_to_element_transform(T_EG);
  else
    T_EG_ = T_EG;
}

void CollisionElementImpl::join_clique(int clique) {
  if(bullet_pimpl_)
    bullet_pimpl_->join_clique(clique);
  else
    ; // Add clique to the vector of cliques.
}

}  // end namespace collision
}  // end namespace drake
