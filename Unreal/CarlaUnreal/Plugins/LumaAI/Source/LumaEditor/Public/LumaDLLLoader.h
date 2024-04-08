// # Copyright 2023 Luma AI
#pragma once
#include "LumaRenderSDKPublic.h"

class LumaDLLLoader
{
private:
	typedef void* (*LUMA_init_t)(void);
	typedef LumaRenderSDK::Gaussians* (*LUMA_load_splat_ply_t)(const char*);
	typedef LumaRenderSDK::GaussianChunks* (*LUMA_UE_chunk_gaussians_t)(LumaRenderSDK::Gaussians*);
	typedef LumaRenderSDK::LumaVolumeField* (*LUMA_load_volume_field_t)(const char*);
	typedef void* (*LUMA_cleanup_t)(void);
public:
	static void FreeDLLHandle();
	static bool LoadDLLHandle();
	static bool LoadFunctions();

	inline static void* DLLHandle;
	inline static LUMA_init_t LUMA_init;
	inline static LUMA_load_splat_ply_t LUMA_load_splat_ply;
	inline static LUMA_UE_chunk_gaussians_t LUMA_UE_chunk_gaussians;
	inline static LUMA_load_volume_field_t LUMA_load_volume_field;
	inline static LUMA_cleanup_t LUMA_cleanup;
};

template <typename T>
struct TLumaDeleter
{
	DECLARE_INLINE_TYPE_LAYOUT(TLumaDeleter, NonVirtual);

	TLumaDeleter() = default;
	TLumaDeleter(const TLumaDeleter&) = default;
	TLumaDeleter& operator=(const TLumaDeleter&) = default;
	~TLumaDeleter() = default;

	template <
		typename U,
		typename = decltype(ImplicitConv<T*>((U*)nullptr))
	>
	TLumaDeleter(const TDefaultDelete<U>&)
	{
	}

	template <
		typename U,
		typename = decltype(ImplicitConv<T*>((U*)nullptr))
	>
	TLumaDeleter& operator=(const TLumaDeleter<U>&)
	{
		return *this;
	}

	void operator()(T* Ptr) const
	{
		Ptr->cleanup();
		free(Ptr);
	}
};

template <typename T, typename Deleter = TLumaDeleter<T>>
class TLumaUniquePtr : private Deleter
{
	DECLARE_INLINE_TYPE_LAYOUT_EXPLICIT_BASES(TLumaUniquePtr, NonVirtual, Deleter);

	template <typename OtherT, typename OtherDeleter>
	friend class TLumaUniquePtr;

public:
	using ElementType = T;

	// Non-copyable
	TLumaUniquePtr(const TLumaUniquePtr&) = delete;
	TLumaUniquePtr& operator=(const TLumaUniquePtr&) = delete;

	/**
	 * Default constructor - initializes the TLumaUniquePtr to null.
	 */
	FORCEINLINE TLumaUniquePtr()
		: Deleter()
		, Ptr    (nullptr)
	{
	}

	/**
	 * Pointer constructor - takes ownership of the pointed-to object
	 *
	 * @param InPtr The pointed-to object to take ownership of.
	 */
	template <
		typename U,
		typename = decltype(ImplicitConv<T*>((U*)nullptr))
	>
	explicit FORCEINLINE TLumaUniquePtr(U* InPtr)
		: Deleter()
		, Ptr    (InPtr)
	{
	}

	/**
	 * Pointer constructor - takes ownership of the pointed-to object
	 *
	 * @param InPtr The pointed-to object to take ownership of.
	 */
	template <
		typename U,
		typename = decltype(ImplicitConv<T*>((U*)nullptr))
	>
	explicit FORCEINLINE TLumaUniquePtr(U* InPtr, Deleter&& InDeleter)
		: Deleter(MoveTemp(InDeleter))
		, Ptr    (InPtr)
	{
	}

	/**
	 * Pointer constructor - takes ownership of the pointed-to object
	 *
	 * @param InPtr The pointed-to object to take ownership of.
	 */
	template <
		typename U,
		typename = decltype(ImplicitConv<T*>((U*)nullptr))
	>
	explicit FORCEINLINE TLumaUniquePtr(U* InPtr, const Deleter& InDeleter)
		: Deleter(InDeleter)
		, Ptr    (InPtr)
	{
	}

	/**
	 * nullptr constructor - initializes the TLumaUniquePtr to null.
	 */
	FORCEINLINE TLumaUniquePtr(TYPE_OF_NULLPTR)
		: Deleter()
		, Ptr    (nullptr)
	{
	}

	/**
	 * Move constructor
	 */
	FORCEINLINE TLumaUniquePtr(TLumaUniquePtr&& Other)
		: Deleter(MoveTemp(Other.GetDeleter()))
		, Ptr    (Other.Ptr)
	{
		Other.Ptr = nullptr;
	}

	/**
	 * Constructor from rvalues of other (usually derived) types
	 */
	template <
		typename OtherT,
		typename OtherDeleter,
		typename = typename TEnableIf<!TIsArray<OtherT>::Value>::Type,
		typename = decltype(ImplicitConv<T*>((OtherT*)nullptr))
	>
	FORCEINLINE TLumaUniquePtr(TLumaUniquePtr<OtherT, OtherDeleter>&& Other)
		: Deleter(MoveTemp(Other.GetDeleter()))
		, Ptr    (Other.Ptr)
	{
		Other.Ptr = nullptr;
	}

	/**
	 * Move assignment operator
	 */
	FORCEINLINE TLumaUniquePtr& operator=(TLumaUniquePtr&& Other)
	{
		if (this != &Other)
		{
			// We delete last, because we don't want odd side effects if the destructor of T relies on the state of this or Other
			T* OldPtr = Ptr;
			Ptr = Other.Ptr;
			Other.Ptr = nullptr;
			GetDeleter()(OldPtr);
		}

		GetDeleter() = MoveTemp(Other.GetDeleter());

		return *this;
	}

	/**
	 * Assignment operator for rvalues of other (usually derived) types
	 */
	template <
		typename OtherT,
		typename OtherDeleter,
		typename = typename TEnableIf<!TIsArray<OtherT>::Value>::Type,
		typename = decltype(ImplicitConv<T*>((OtherT*)nullptr))
	>
	FORCEINLINE TLumaUniquePtr& operator=(TLumaUniquePtr<OtherT, OtherDeleter>&& Other)
	{
		// We delete last, because we don't want odd side effects if the destructor of T relies on the state of this or Other
		T* OldPtr = Ptr;
		Ptr = Other.Ptr;
		Other.Ptr = nullptr;
		GetDeleter()(OldPtr);

		GetDeleter() = MoveTemp(Other.GetDeleter());

		return *this;
	}

	/**
	 * Nullptr assignment operator
	 */
	FORCEINLINE TLumaUniquePtr& operator=(TYPE_OF_NULLPTR)
	{
		// We delete last, because we don't want odd side effects if the destructor of T relies on the state of this
		T* OldPtr = Ptr;
		Ptr = nullptr;
		GetDeleter()(OldPtr);

		return *this;
	}

	/**
	 * Destructor
	 */
	FORCEINLINE ~TLumaUniquePtr()
	{
		GetDeleter()(Ptr);
	}

	/**
	 * Tests if the TLumaUniquePtr currently owns an object.
	 *
	 * @return true if the TLumaUniquePtr currently owns an object, false otherwise.
	 */
	bool IsValid() const
	{
		return Ptr != nullptr;
	}

	/**
	 * operator bool
	 *
	 * @return true if the TLumaUniquePtr currently owns an object, false otherwise.
	 */
	FORCEINLINE explicit operator bool() const
	{
		return IsValid();
	}

	/**
	 * Logical not operator
	 *
	 * @return false if the TLumaUniquePtr currently owns an object, true otherwise.
	 */
	FORCEINLINE bool operator!() const
	{
		return !IsValid();
	}

	/**
	 * Indirection operator
	 *
	 * @return A pointer to the object owned by the TLumaUniquePtr.
	 */
	FORCEINLINE T* operator->() const
	{
		return Ptr;
	}

	/**
	 * Dereference operator
	 *
	 * @return A reference to the object owned by the TLumaUniquePtr.
	 */
	FORCEINLINE T& operator*() const
	{
		return *Ptr;
	}

	/**
	 * Returns a pointer to the owned object without relinquishing ownership.
	 *
	 * @return A copy of the pointer to the object owned by the TLumaUniquePtr, or nullptr if no object is being owned.
	 */
	FORCEINLINE T* Get() const
	{
		return Ptr;
	}

	/**
	 * Relinquishes control of the owned object to the caller and nulls the TLumaUniquePtr.
	 *
	 * @return The pointer to the object that was owned by the TLumaUniquePtr, or nullptr if no object was being owned.
	 */
	FORCEINLINE T* Release()
	{
		T* Result = Ptr;
		Ptr = nullptr;
		return Result;
	}

	/**
	 * Gives the TLumaUniquePtr a new object to own, destroying any previously-owned object.
	 *
	 * @param InPtr A pointer to the object to take ownership of.
	 */
	FORCEINLINE void Reset(T* InPtr = nullptr)
	{
		if (Ptr != InPtr)
		{
			// We delete last, because we don't want odd side effects if the destructor of T relies on the state of this
			T* OldPtr = Ptr;
			Ptr = InPtr;
			GetDeleter()(OldPtr);
		}
	}

	/**
	 * Returns a reference to the deleter subobject.
	 *
	 * @return A reference to the deleter.
	 */
	FORCEINLINE Deleter& GetDeleter()
	{
		return static_cast<Deleter&>(*this);
	}

	/**
	 * Returns a reference to the deleter subobject.
	 *
	 * @return A reference to the deleter.
	 */
	FORCEINLINE const Deleter& GetDeleter() const
	{
		return static_cast<const Deleter&>(*this);
	}

private:
	using PtrType = T*;
	LAYOUT_FIELD(PtrType, Ptr);
};