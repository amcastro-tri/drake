#pragma once

#include "drake/common/drake_assert.h"

namespace drake {
namespace multibody {

// Forward declaration.
template<typename T> class MultibodyTree;

// Primary template defining the signature of this class.
// Two template arguments are required in general, the type of the class
// inheriting from MultibodyTreeElement (i.e. this is a CRTP class), and the
// type of the type-safe index used to identify these element types within a
// MultibodyTree.
// The signature below is an explicit (full) template specialization for the
// case in which the class is a template in a scalar type T. The template
// specialization allows the compiler to automatically deduce the scalar type
// from the subclass template signature itself.
template <class ElementType, typename ElementIndexType>
class MultibodyTreeElement;

/// A class representing an element or component of a MultibodyTree. Examples of
/// multibody tree elements are bodies, joints, force elements, and constraints.
/// Multibody tree elements are owned and managed by a parent MultibodyTree.
/// As part of their construction process they get assigned an id that uniquely
/// identifies them within their parent MultibodyTree.
/// A generic multibody tree element `MultibodyComponent` is derived from
/// this class as: <pre>
/// template <typename T>
/// class MultibodyComponent :
///     public MultibodyTreeElement<MultibodyComponent<T>,
///                                 MultibodyComponentIndex> {
///  ...
/// };
/// </pre>
///
/// @tparam ElementType The type of the specific multibody element, for
///                     instance, a body or a mobilizer. It must be a template
///                     class on the scalar type `T`.
/// @tparam T The underlying scalar type. Must be a valid Eigen scalar. With the
///           signature below the scalar type is automatically deduced from the
///           `ElementType` template argument.
/// @tparam ElemeentIndexType The type-safe index used for this element type.
///
/// As an example of usage, consider the definition of a `ForceElement` class
/// as a multibody tree element. This would be accomplished with: <pre>
///   template <typename T>
///   class ForceElement :
///       public MultibodyTreeElement<ForceElement<T>, BodyIndex>;
/// </pre>
/// Notice that the with the signature below the scalar type is automatically
/// deduced from the template arguments.
template <template <typename> class ElementType,
    typename T, typename ElementIndexType>
class MultibodyTreeElement<ElementType<T>, ElementIndexType> {
 public:
  virtual ~MultibodyTreeElement() {}

  /// Returns a constant reference to the parent MultibodyTree that owns
  /// this element.
  /// By construction a %MultibodyTreeElement **always** has a parent
  /// MultibodyTree. This method, however, asserts that this is the case in
  /// Debug builds.
  const MultibodyTree<T>& get_parent_tree() const {
    DRAKE_ASSERT_VOID(HasParentTreeOrThrow());
    return *parent_tree_;
  }

  /// Returns the unique identifier in its parent MultibodyTree to this element.
  ElementIndexType get_index() const { return id_;}

  /// Checks whether this MultibodyTreeElement has been registered into a
  /// MultibodyTree. If not, it throws an exception of type std::logic_error.
  void HasParentTreeOrThrow() const {
    if (parent_tree_ == nullptr) {
      throw std::logic_error(
          "This multibody component was not added to a MultibodyTree.");
    }
  }

  /// Checks whether `this` element has the same parent tree as @p other.
  /// If not, it throws an exception of type std::logic_error.
  template <template <typename> class OtherElementType,
      typename OtherElementIndexType>
  void HasSameParentTreeOrThrow(
      const MultibodyTreeElement<OtherElementType<T>, OtherElementIndexType>&
      other) const {
    this->HasParentTreeOrThrow();
    other.HasParentTreeOrThrow();
    if (parent_tree_ != other.parent_tree_) {
      throw std::logic_error(
          "These two MultibodyTreeElement's do not belong to "
          "the same MultibodyTree.");
    }
  }

  /// Gives MultibodyTree elements the opportunity to perform internal setup
  /// when MultibodyTree::Compile() is invoked.
  virtual void Compile() = 0;

  // TODO(amcastro-tri): Add DeepClone API for transmogrification to other
  // scalar types.
  // This will make use the template argument "MultibodyTreeElement".

 protected:
  const MultibodyTree<T>* parent_tree_{nullptr};
  ElementIndexType id_{0};  // ElementIndexType requires a valid initialization.

  // Only derived sub-classes can call these set methods from within their
  // Create() factories.
  void set_parent_tree(const MultibodyTree<T>* tree) { parent_tree_ = tree; }
  virtual void set_id(ElementIndexType id) { id_ = id; }
};

}  // namespace multibody
}  // namespace drake
