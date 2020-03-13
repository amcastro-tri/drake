#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/force_element.h"

namespace drake {
namespace multibody {

template <typename T>
class CustomForceElement final : public ForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CustomForceElement)

  CustomForceElement(ModelInstanceIndex model_instance);

  virtual T CalcPotentialEnergy(const systems::Context<T>&) const {
    throw std::logic_error("CalcPotentialEnergy() not implemented.");
  }

  virtual T CalcConservativePower(const systems::Context<T>&) const {
    throw std::logic_error("CalcConservativePower() not implemented.");
  }

  virtual T CalcNonConservativePower(const systems::Context<T>&) const {
    throw std::logic_error("CalcNonConservativePower() not implemented.");
  }

 protected:
  virtual void DoCalcAndAddForceContribution(
      const systems::Context<T>& context, MultibodyForces<T>* forces) const = 0;

  virtual std::unique_ptr<CustomForceElement<double>> ToDouble() const {
    throw std::logic_error("This element dos not support ToDouble().");
  }

  virtual std::unique_ptr<CustomForceElement<AutoDiffXd>> ToAutoDiffXd() const {
    throw std::logic_error("This element dos not support ToAutoDiffXd().");
  }

  virtual std::unique_ptr<CustomForceElement<symbolic::Expression>> ToSymbolic()
      const {
    throw std::logic_error("This element dos not support ToSymbolic().");
  }

 private:  
  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U> friend class CustomForceElement;

  T CalcPotentialEnergy(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc) const final;

  T CalcConservativePower(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc) const final;

  T CalcNonConservativePower(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc) const final;

  void DoCalcAndAddForceContribution(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc,
      MultibodyForces<T>* forces) const final;

  std::unique_ptr<ForceElement<double>> DoCloneToScalar(
      const internal::MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<ForceElement<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const override;

  std::unique_ptr<ForceElement<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>&) const override;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::CustomForceElement)
