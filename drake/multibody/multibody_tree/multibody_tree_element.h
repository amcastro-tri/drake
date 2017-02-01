#pragma once

namespace drake {
namespace multibody {

// Forward declaration.
template<typename T> class MultibodyTree;

template <class ElementType, typename ElementIndexType>
class MultibodyTreeElement;

template <template <typename> class ElementType, typename T,
    typename ElementIndexType>
class MultibodyTreeElement<ElementType<T>, ElementIndexType> {
 public:
  friend MultibodyTree<T>;

  const MultibodyTree<T>& get_parent_tree() const {
    if (parent_tree_ == nullptr || id_.is_invalid())
      throw std::runtime_error(
          "This multibody component was not added to a MultibodyTree.");
    return *parent_tree_;
  }

  MultibodyTree<T>* get_mutable_parent_tree() const {
    if (parent_tree_ == nullptr || id_.is_invalid())
      throw std::runtime_error(
          "This multibody component was not added to a MultibodyTree.");
    return parent_tree_;
  }

  ElementIndexType get_id() const { return id_;}

 private:
  MultibodyTree<T>* parent_tree_;
  ElementIndexType id_{ElementIndexType::Invalid()};

  // Only MultibodyTree<T> can set these.
  void set_parent_tree(MultibodyTree<T>* tree) { parent_tree_ = tree; }
  void set_id(ElementIndexType id) { id_ = id; }

#if 0
  // Non-virtual interface (NVI) to DoDeepClone(). Derived classes must
  // overwrite this method to allow the deep cloning of MultibodyTree to other
  // scalar types.
  template <typename ToScalar>
  ElementType<ToScalar>* DeepCloneTo() const {
    return ElementType<ToScalar>::DoDeepClone(*this);
  };

  // Derived classes must overwrite this method to allow the deep cloning of
  // MultibodyTree to other scalar types.
  virtual ElementType<T>* DoDeepCloneTo(const ElementType<double>& other) const
  {
    return nullptr;
  };
#endif

};

}  // namespace multibody
}  // namespace drake
