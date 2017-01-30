#pragma once

namespace drake {
namespace multibody {

// Forward declaration.
template<typename T> class MultibodyTree;

template <class MemberType, typename MemberIndexType>
class MultibodyTreeMember;

template <template <typename> class MemberType, typename T,
    typename MemberIndexType>
class MultibodyTreeMember<MemberType<T>, MemberIndexType> {
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

  MemberIndexType get_id() const { return id_;}

 private:
  MultibodyTree<T>* parent_tree_;
  MemberIndexType id_{MemberIndexType::Invalid()};

  // Only MultibodyTree<T> can set these.
  void set_parent_tree(MultibodyTree<T>* tree) { parent_tree_ = tree; }
  void set_id(MemberIndexType id) { id_ = id; }
};

}  // namespace multibody
}  // namespace drake
