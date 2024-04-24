#include <cassert>
#include <iostream>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/nice_type_name.h"

/// NOTE: This is just a concept experiments. Separators are used to show how
/// this file would be split into smaller files. Also, the order of includes
/// (the order of declarations here) matter. Also, names are simply for
/// demonstration. The actual code won't have JointBase<T>, but MobodBase<T>,
/// etc.


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// File joint_base.h
////////////////////////////////////////////////////////////////////////////////

/// Virtual interface for all joint types.
class JointBase {
 public:
  virtual void PrintName() const = 0;
  virtual ~JointBase() = default;
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// File joint_impl.h
////////////////////////////////////////////////////////////////////////////////

/// CRTP type that implements the JointBase interface. Code that is shared
/// across joints will live here, optimized via CRTP and traits to specfic joint
/// types. In other words, this CRTP help us generate code with a common
/// structure (though maybe with different types and sizes) throuout joint
/// types. Calls within the scope of functions in this class, can be inlined by
/// the compiler. Therfore calls from JointBase are only reserved for high level
/// loops (say the kernel in the real code) so that polymorphic calls must only
/// be resolved once per type.
template <class Derived>
class JointImpl : public JointBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(JointImpl)

  JointImpl() = default;

  // JointImpl() : Base(std::in_place_type<Derived>) {}

  /// Prints the NiceTypeName of this joint's type.
  /// @pre Derived::DoPrintName() is defined.
  void PrintName() const override {
    std::cout << "JointImpl<" << drake::NiceTypeName::Get<Derived>() << ">"
              << std::endl;
    /// Call type specific implementation. This call, and others if made here,
    /// can be inlined within this scope.
    derived().DoPrintName();
    std::cout << std::endl;
  }

 protected:
  const Derived& derived() const { return *static_cast<const Derived*>(this); }
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// File joint_one.h
////////////////////////////////////////////////////////////////////////////////

/// A specific joint type.
class JointOne final : public JointImpl<JointOne> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(JointOne)

  JointOne() {}

  JointOne(int data) : data_(data) {}

 private:
  // Give JointImpl access to private implementations.
  friend JointImpl<JointOne>;
  void DoPrintName() const {
    std::cout << "JointOne with data = " << data_ << "\n";
  }
  int data_;  // dummy data.
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// File joint_two.h
////////////////////////////////////////////////////////////////////////////////

/// Another joint type, see notes in JointOne.
class JointTwo : public JointImpl<JointTwo> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(JointTwo)

  JointTwo() = default;

  JointTwo(double d) : data_(d) {}

 private:
  friend JointImpl<JointTwo>;
  void DoPrintName() const {
    std::cout << "JointTwo with data = " << data_ << "\n";
  }
  double data_;
  double data2_;
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// File type_list.h
////////////////////////////////////////////////////////////////////////////////

template <typename... Ts>
struct SizeOfList;

template <template <typename...> class TL, typename... Ts>
struct SizeOfList<TL<Ts...>> {
  constexpr static auto value = sizeof...(Ts);
};

template <class... Ts>
constexpr size_t TypeListMaxSize() noexcept {
  size_t maxSize{0};
  ((maxSize = std::max(maxSize, sizeof(Ts))), ...);
  return maxSize;
}

// A structs that represents a compile-time list of types.
template <typename...>
struct TypeList;

// Compile-time size of a TypeList.
template <typename TList>
struct Length;

template <typename... Types>
struct Length<TypeList<Types...>> {
  static constexpr std::size_t value = sizeof...(Types);
};

// Retrieves the type T in TList with the largest sizeof(T).
template <typename TList>
struct LargestType;

// Specialization for a list with a single type.
template <typename T>
struct LargestType<TypeList<T>> {
  using type = T;
};

// Recursive specialization for lists with more than one type.
template <typename T, typename U, typename... Ts>
struct LargestType<TypeList<T, U, Ts...>> {
  using type = typename LargestType<
      TypeList<typename std::conditional<(sizeof(U) <= sizeof(T)), T, U>::type,
               Ts...>>::type;
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// File pool.h
////////////////////////////////////////////////////////////////////////////////

/// Pool is a container of objects with types that belong to a closed set of
/// types, specified by TList.
///
/// Objects in a pool are stored by value, contiguously in memory, in order to
/// minimize cache misses when traversing elements in the pool.
/// To accomodate objects with types of different sizes, each value is stored
/// in a bucket of size equal to the maximum type size in TList.
///
/// Objects in TList are polymorphic, with base class TBase, even when stored
/// contiguously in memory by value.
///
/// @tparam TList a TypeList that specifies the list of types.
/// @tparam TBase the base class of polymorphic objects in TList.
template <typename TList, typename TBase>
class Pool {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Pool)

  /// Constructs a pool with a given `capacity`.
  /// After this call, we cannot place more than `capacity` objects in the pool.
  explicit Pool(int capacity) { pool_.reserve(capacity); }

  /// Constructs an object of type U directly in the pool, forwarding arguments
  /// `args`.
  ///
  /// A call to this method looks like:
  ///  pool.emplace_back<Type>(arg1, arg2, ...);
  ///
  /// @pre size() < capacity().
  /// @pre U must be a type in TList.
  /// @pre U must inherit from TBase.
  template <typename U, class... Args>
  void emplace_back(Args&&... args) {
    DRAKE_DEMAND(size() < capacity());
    static_assert(std::is_base_of_v<TBase, U>);
    // TODO: Some template code or SFINAE to bark if U is not one of the
    // supported types.
    pool_.push_back(Bucket{});  // un-initialized bucket, increases vector size.
    new (&pool_.back())
        U(std::forward<Args>(args)...);  // Placement new into the new bucket.
  }

  /// The maximum number of objects this pool can store.
  int capacity() const { return static_cast<int>(pool_.capacity()); }

  /// The current number of objects stored by the pool.
  int size() const { return ssize(pool_); }

  /// Returns a refernce to the common interface for the i-th object.
  const JointBase& operator[](int i) const {
    return *reinterpret_cast<const JointBase*>(&pool_[i]);
  }

 private:
  /// The pool stores equal sized buckets. The size is determined at
  /// compile-time as the maximum sizeof(T) with T in TList.
  struct Bucket {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Bucket)
    constexpr static int kBucketSize =
        sizeof(typename LargestType<TList>::type);
    Bucket() = default;
    std::byte storage_[kBucketSize];
  };
  std::vector<Bucket> pool_;  // The actual pool of objects.
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// Application file
////////////////////////////////////////////////////////////////////////////////

int main() {
  /// Compile-time list of the closed set of joints we'll work with.
  using JointsList = TypeList<JointOne, JointTwo>;

  /// Some experiments with compile-time information.
  std::cout << "List size: " << Length<JointsList>::value << std::endl;
  std::cout << "sizeof(JointOne): " << sizeof(JointOne) << std::endl;
  std::cout << "sizeof(JointTwo): " << sizeof(JointTwo) << std::endl;
  std::cout << "The largest type is: "
            << drake::NiceTypeName::Get<LargestType<JointsList>::type>()
            << std::endl;

  /// Create a pool for thre joints, push some joints, and print their names to
  /// test the virtual dispatching is working properly.
  using JointPool = Pool<JointsList, JointBase>;
  JointPool pool(3);
  pool.emplace_back<JointTwo>();
  pool.emplace_back<JointOne>(1 /* data */);
  pool.emplace_back<JointTwo>(3.14 /* data */);
  for (int i = 0; i < pool.size(); ++i) {
    pool[i].PrintName();
  }
}
