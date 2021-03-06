/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef _LINUX_XARRAY_H
#define _LINUX_XARRAY_H
/*
 * eXtensible Arrays
 * Copyright (c) 2017 Microsoft Corporation
 * Author: Matthew Wilcox <willy@infradead.org>
 *
 * See Documentation/core-api/xarray.rst for how to use the XArray.
 */

#include <linux/bug.h>
#include <linux/compiler.h>
#include <linux/gfp.h>
#include <linux/kconfig.h>
#include <linux/kernel.h>
#include <linux/rcupdate.h>
#include <linux/spinlock.h>
#include <linux/types.h>

/*
 * The bottom two bits of the entry determine how the XArray interprets
 * the contents:
 *
 * 00: Pointer entry
 * 10: Internal entry
 * x1: Value entry
 *
 * Attempting to store internal entries in the XArray is a bug.
 *
 * Most internal entries are pointers to the next node in the tree.
 * The following internal entries have a special meaning:
 *
 * 0-62: Sibling entries
 * 256: Retry entry
 *
 * Errors are also represented as internal entries, but use the negative
 * space (-4094 to -2).  They're never stored in the slots array; only
 * returned by the normal API.
 */

#define BITS_PER_XA_VALUE	(BITS_PER_LONG - 1)

/**
 * xa_mk_value() - Create an XArray entry from an integer.
 * @v: Value to store in XArray.
 *
 * Context: Any context.
 * Return: An entry suitable for storing in the XArray.
 */
static inline void *xa_mk_value(unsigned long v)
{
	WARN_ON((long)v < 0);
	return (void *)((v << 1) | 1);
}

/**
 * xa_to_value() - Get value stored in an XArray entry.
 * @entry: XArray entry.
 *
 * Context: Any context.
 * Return: The value stored in the XArray entry.
 */
static inline unsigned long xa_to_value(const void *entry)
{
	return (unsigned long)entry >> 1;
}

/**
 * xa_is_value() - Determine if an entry is a value.
 * @entry: XArray entry.
 *
 * Context: Any context.
 * Return: True if the entry is a value, false if it is a pointer.
 */
static inline bool xa_is_value(const void *entry)
{
	return (unsigned long)entry & 1;
}

/*
 * xa_mk_internal() - Create an internal entry.
 * @v: Value to turn into an internal entry.
 *
 * Context: Any context.
 * Return: An XArray internal entry corresponding to this value.
 */
static inline void *xa_mk_internal(unsigned long v)
{
	return (void *)((v << 2) | 2);
}

/*
 * xa_to_internal() - Extract the value from an internal entry.
 * @entry: XArray entry.
 *
 * Context: Any context.
 * Return: The value which was stored in the internal entry.
 */
static inline unsigned long xa_to_internal(const void *entry)
{
	return (unsigned long)entry >> 2;
}

/*
 * xa_is_internal() - Is the entry an internal entry?
 * @entry: XArray entry.
 *
 * Context: Any context.
 * Return: %true if the entry is an internal entry.
 */
static inline bool xa_is_internal(const void *entry)
{
	return ((unsigned long)entry & 3) == 2;
}

/**
 * xa_is_err() - Report whether an XArray operation returned an error
 * @entry: Result from calling an XArray function
 *
 * If an XArray operation cannot complete an operation, it will return
 * a special value indicating an error.  This function tells you
 * whether an error occurred; xa_err() tells you which error occurred.
 *
 * Context: Any context.
 * Return: %true if the entry indicates an error.
 */
static inline bool xa_is_err(const void *entry)
{
	return unlikely(xa_is_internal(entry));
}

/**
 * xa_err() - Turn an XArray result into an errno.
 * @entry: Result from calling an XArray function.
 *
 * If an XArray operation cannot complete an operation, it will return
 * a special pointer value which encodes an errno.  This function extracts
 * the errno from the pointer value, or returns 0 if the pointer does not
 * represent an errno.
 *
 * Context: Any context.
 * Return: A negative errno or 0.
 */
static inline int xa_err(void *entry)
{
	/* xa_to_internal() would not do sign extension. */
	if (xa_is_err(entry))
		return (long)entry >> 2;
	return 0;
}

typedef unsigned __bitwise xa_tag_t;
#define XA_TAG_0		((__force xa_tag_t)0U)
#define XA_TAG_1		((__force xa_tag_t)1U)
#define XA_TAG_2		((__force xa_tag_t)2U)
#define XA_PRESENT		((__force xa_tag_t)8U)
#define XA_TAG_MAX		XA_TAG_2

enum xa_lock_type {
	XA_LOCK_IRQ = 1,
	XA_LOCK_BH = 2,
};

/*
 * Values for xa_flags.  The radix tree stores its GFP flags in the xa_flags,
 * and we remain compatible with that.
 */
#define XA_FLAGS_LOCK_IRQ	((__force gfp_t)XA_LOCK_IRQ)
#define XA_FLAGS_LOCK_BH	((__force gfp_t)XA_LOCK_BH)
#define XA_FLAGS_TAG(tag)	((__force gfp_t)((1U << __GFP_BITS_SHIFT) << \
						(__force unsigned)(tag)))

/**
 * struct xarray - The anchor of the XArray.
 * @xa_lock: Lock that protects the contents of the XArray.
 *
 * To use the xarray, define it statically or embed it in your data structure.
 * It is a very small data structure, so it does not usually make sense to
 * allocate it separately and keep a pointer to it in your data structure.
 *
 * You may use the xa_lock to protect your own data structures as well.
 */
/*
 * If all of the entries in the array are NULL, @xa_head is a NULL pointer.
 * If the only non-NULL entry in the array is at index 0, @xa_head is that
 * entry.  If any other entry in the array is non-NULL, @xa_head points
 * to an @xa_node.
 */
struct xarray {
	spinlock_t	xa_lock;
/* private: The rest of the data structure is not to be used directly. */
	gfp_t		xa_flags;
	void __rcu *	xa_head;
};

#define XARRAY_INIT_FLAGS(name, flags) {			\
	.xa_lock = __SPIN_LOCK_UNLOCKED(name.xa_lock),		\
	.xa_flags = flags,					\
	.xa_head = NULL,					\
}

#define XARRAY_INIT(name) XARRAY_INIT_FLAGS(name, 0)

/**
 * DEFINE_XARRAY() - Define an XArray
 * @name: A string that names your XArray
 *
 * This is intended for file scope definitions of XArrays.  It declares
 * and initialises an empty XArray with the chosen name.  It is equivalent
 * to calling xa_init() on the array, but it does the initialisation at
 * compiletime instead of runtime.
 */
#define DEFINE_XARRAY(name) struct xarray name = XARRAY_INIT(name)
#define DEFINE_XARRAY_FLAGS(name, flags) \
			struct xarray name = XARRAY_INIT_FLAGS(name, flags)

void xa_init_flags(struct xarray *, gfp_t flags);
void *xa_load(struct xarray *, unsigned long index);
void *xa_store(struct xarray *, unsigned long index, void *entry, gfp_t);
void *xa_cmpxchg(struct xarray *, unsigned long index,
			void *old, void *entry, gfp_t);
bool xa_get_tag(struct xarray *, unsigned long index, xa_tag_t);
void xa_set_tag(struct xarray *, unsigned long index, xa_tag_t);
void xa_clear_tag(struct xarray *, unsigned long index, xa_tag_t);
void *xa_find(struct xarray *xa, unsigned long *index,
		unsigned long max, xa_tag_t) __attribute__((nonnull(2)));
void *xa_find_after(struct xarray *xa, unsigned long *index,
		unsigned long max, xa_tag_t) __attribute__((nonnull(2)));
unsigned int xa_extract(struct xarray *, void **dst, unsigned long start,
		unsigned long max, unsigned int n, xa_tag_t);
void xa_destroy(struct xarray *);

/**
 * xa_init() - Initialise an empty XArray.
 * @xa: XArray.
 *
 * An empty XArray is full of NULL entries.
 *
 * Context: Any context.
 */
static inline void xa_init(struct xarray *xa)
{
	xa_init_flags(xa, 0);
}

/**
 * xa_empty() - Determine if an array has any present entries.
 * @xa: XArray.
 *
 * Context: Any context.
 * Return: %true if the array contains only NULL pointers.
 */
static inline bool xa_empty(const struct xarray *xa)
{
	return xa->xa_head == NULL;
}

/**
 * xa_tagged() - Inquire whether any entry in this array has a tag set
 * @xa: Array
 * @tag: Tag value
 *
 * Context: Any context.
 * Return: %true if any entry has this tag set.
 */
static inline bool xa_tagged(const struct xarray *xa, xa_tag_t tag)
{
	return xa->xa_flags & XA_FLAGS_TAG(tag);
}

/**
 * xa_erase() - Erase this entry from the XArray.
 * @xa: XArray.
 * @index: Index of entry.
 *
 * This function is the equivalent of calling xa_store() with %NULL as
 * the third argument.  The XArray does not need to allocate memory, so
 * the user does not need to provide GFP flags.
 *
 * Context: Process context.  Takes and releases the xa_lock.
 * Return: The entry which used to be at this index.
 */
static inline void *xa_erase(struct xarray *xa, unsigned long index)
{
	return xa_store(xa, index, NULL, 0);
}

/**
 * xa_insert() - Store this entry in the XArray unless another entry is
 *			already present.
 * @xa: XArray.
 * @index: Index into array.
 * @entry: New entry.
 * @gfp: Memory allocation flags.
 *
 * If you would rather see the existing entry in the array, use xa_cmpxchg().
 * This function is for users who don't care what the entry is, only that
 * one is present.
 *
 * Context: Process context.  Takes and releases the xa_lock.
 *	    May sleep if the @gfp flags permit.
 * Return: 0 if the store succeeded.  -EEXIST if another entry was present.
 * 	   -ENOMEM if memory could not be allocated.
 */
static inline int xa_insert(struct xarray *xa, unsigned long index,
		void *entry, gfp_t gfp)
{
	void *curr = xa_cmpxchg(xa, index, NULL, entry, gfp);
	if (!curr)
		return 0;
	if (xa_is_err(curr))
		return xa_err(curr);
	return -EEXIST;
}

/**
 * xa_for_each() - Iterate over a portion of an XArray.
 * @xa: XArray.
 * @entry: Entry retrieved from array.
 * @index: Index of @entry.
 * @max: Maximum index to retrieve from array.
 * @filter: Selection criterion.
 *
 * Initialise @index to the lowest index you want to retrieve from the
 * array.  During the iteration, @entry will have the value of the entry
 * stored in @xa at @index.  The iteration will skip all entries in the
 * array which do not match @filter.  You may modify @index during the
 * iteration if you want to skip or reprocess indices.  It is safe to modify
 * the array during the iteration.  At the end of the iteration, @entry will
 * be set to NULL and @index will have a value less than or equal to max.
 *
 * xa_for_each() is O(n.log(n)) while xas_for_each() is O(n).  You have
 * to handle your own locking with xas_for_each(), and if you have to unlock
 * after each iteration, it will also end up being O(n.log(n)).  xa_for_each()
 * will spin if it hits a retry entry; if you intend to see retry entries,
 * you should use the xas_for_each() iterator instead.  The xas_for_each()
 * iterator will expand into more inline code than xa_for_each().
 *
 * Context: Any context.  Takes and releases the RCU lock.
 */
#define xa_for_each(xa, entry, index, max, filter) \
	for (entry = xa_find(xa, &index, max, filter); entry; \
	     entry = xa_find_after(xa, &index, max, filter))

#define xa_trylock(xa)		spin_trylock(&(xa)->xa_lock)
#define xa_lock(xa)		spin_lock(&(xa)->xa_lock)
#define xa_unlock(xa)		spin_unlock(&(xa)->xa_lock)
#define xa_lock_bh(xa)		spin_lock_bh(&(xa)->xa_lock)
#define xa_unlock_bh(xa)	spin_unlock_bh(&(xa)->xa_lock)
#define xa_lock_irq(xa)		spin_lock_irq(&(xa)->xa_lock)
#define xa_unlock_irq(xa)	spin_unlock_irq(&(xa)->xa_lock)
#define xa_lock_irqsave(xa, flags) \
				spin_lock_irqsave(&(xa)->xa_lock, flags)
#define xa_unlock_irqrestore(xa, flags) \
				spin_unlock_irqrestore(&(xa)->xa_lock, flags)

/*
 * Versions of the normal API which require the caller to hold the xa_lock.
 * If the GFP flags allow it, will drop the lock in order to allocate
 * memory, then reacquire it afterwards.
 */
void *__xa_erase(struct xarray *, unsigned long index);
void *__xa_store(struct xarray *, unsigned long index, void *entry, gfp_t);
void *__xa_cmpxchg(struct xarray *, unsigned long index, void *old,
		void *entry, gfp_t);
void __xa_set_tag(struct xarray *, unsigned long index, xa_tag_t);
void __xa_clear_tag(struct xarray *, unsigned long index, xa_tag_t);

/**
 * __xa_insert() - Store this entry in the XArray unless another entry is
 *			already present.
 * @xa: XArray.
 * @index: Index into array.
 * @entry: New entry.
 * @gfp: Memory allocation flags.
 *
 * If you would rather see the existing entry in the array, use __xa_cmpxchg().
 * This function is for users who don't care what the entry is, only that
 * one is present.
 *
 * Context: Any context.  Expects xa_lock to be held on entry.  May
 *	    release and reacquire xa_lock if the @gfp flags permit.
 * Return: 0 if the store succeeded.  -EEXIST if another entry was present.
 *	   -ENOMEM if memory could not be allocated.
 */
static inline int __xa_insert(struct xarray *xa, unsigned long index,
		void *entry, gfp_t gfp)
{
	void *curr = __xa_cmpxchg(xa, index, NULL, entry, gfp);
	if (!curr)
		return 0;
	if (xa_is_err(curr))
		return xa_err(curr);
	return -EEXIST;
}

/* Everything below here is the Advanced API.  Proceed with caution. */

/*
 * The xarray is constructed out of a set of 'chunks' of pointers.  Choosing
 * the best chunk size requires some tradeoffs.  A power of two recommends
 * itself so that we can walk the tree based purely on shifts and masks.
 * Generally, the larger the better; as the number of slots per level of the
 * tree increases, the less tall the tree needs to be.  But that needs to be
 * balanced against the memory consumption of each node.  On a 64-bit system,
 * xa_node is currently 576 bytes, and we get 7 of them per 4kB page.  If we
 * doubled the number of slots per node, we'd get only 3 nodes per 4kB page.
 */
#ifndef XA_CHUNK_SHIFT
#define XA_CHUNK_SHIFT		(CONFIG_BASE_SMALL ? 4 : 6)
#endif
#define XA_CHUNK_SIZE		(1UL << XA_CHUNK_SHIFT)
#define XA_CHUNK_MASK		(XA_CHUNK_SIZE - 1)
#define XA_MAX_TAGS		3
#define XA_TAG_LONGS		DIV_ROUND_UP(XA_CHUNK_SIZE, BITS_PER_LONG)

/*
 * @count is the count of every non-NULL element in the ->slots array
 * whether that is a value entry, a retry entry, a user pointer,
 * a sibling entry or a pointer to the next level of the tree.
 * @nr_values is the count of every element in ->slots which is
 * either a value entry or a sibling entry to a value entry.
 */
struct xa_node {
	unsigned char	shift;		/* Bits remaining in each slot */
	unsigned char	offset;		/* Slot offset in parent */
	unsigned char	count;		/* Total entry count */
	unsigned char	nr_values;	/* Value entry count */
	struct xa_node __rcu *parent;	/* NULL at top of tree */
	struct xarray	*array;		/* The array we belong to */
	union {
		struct list_head private_list;	/* For tree user */
		struct rcu_head	rcu_head;	/* Used when freeing node */
	};
	void __rcu	*slots[XA_CHUNK_SIZE];
	unsigned long	tags[XA_MAX_TAGS][XA_TAG_LONGS];
};

void xa_dump(const struct xarray *);
void xa_dump_node(const struct xa_node *);

#ifdef XA_DEBUG
#define XA_BUG_ON(xa, x) do {					\
		if (x) {					\
			xa_dump(xa);				\
			BUG();					\
		}						\
	} while (0)
#define XA_NODE_BUG_ON(node, x) do {				\
		if (x) {					\
			if (node) xa_dump_node(node);		\
			BUG();					\
		}						\
	} while (0)
#else
#define XA_BUG_ON(xa, x)	do { } while (0)
#define XA_NODE_BUG_ON(node, x)	do { } while (0)
#endif

/* Private */
static inline void *xa_head(const struct xarray *xa)
{
	return rcu_dereference_check(xa->xa_head,
						lockdep_is_held(&xa->xa_lock));
}

/* Private */
static inline void *xa_head_locked(const struct xarray *xa)
{
	return rcu_dereference_protected(xa->xa_head,
						lockdep_is_held(&xa->xa_lock));
}

/* Private */
static inline void *xa_entry(const struct xarray *xa,
				const struct xa_node *node, unsigned int offset)
{
	XA_NODE_BUG_ON(node, offset >= XA_CHUNK_SIZE);
	return rcu_dereference_check(node->slots[offset],
						lockdep_is_held(&xa->xa_lock));
}

/* Private */
static inline void *xa_entry_locked(const struct xarray *xa,
				const struct xa_node *node, unsigned int offset)
{
	XA_NODE_BUG_ON(node, offset >= XA_CHUNK_SIZE);
	return rcu_dereference_protected(node->slots[offset],
						lockdep_is_held(&xa->xa_lock));
}

/* Private */
static inline struct xa_node *xa_parent(const struct xarray *xa,
					const struct xa_node *node)
{
	return rcu_dereference_check(node->parent,
						lockdep_is_held(&xa->xa_lock));
}

/* Private */
static inline struct xa_node *xa_parent_locked(const struct xarray *xa,
					const struct xa_node *node)
{
	return rcu_dereference_protected(node->parent,
						lockdep_is_held(&xa->xa_lock));
}

/* Private */
static inline void *xa_mk_node(const struct xa_node *node)
{
	return (void *)((unsigned long)node | 2);
}

/* Private */
static inline struct xa_node *xa_to_node(const void *entry)
{
	return (struct xa_node *)((unsigned long)entry - 2);
}

/* Private */
static inline bool xa_is_node(const void *entry)
{
	return xa_is_internal(entry) && (unsigned long)entry > 4096;
}

/* Private */
static inline void *xa_mk_sibling(unsigned int offset)
{
	return xa_mk_internal(offset);
}

/* Private */
static inline unsigned long xa_to_sibling(const void *entry)
{
	return xa_to_internal(entry);
}

/**
 * xa_is_sibling() - Is the entry a sibling entry?
 * @entry: Entry retrieved from the XArray
 *
 * Return: %true if the entry is a sibling entry.
 */
static inline bool xa_is_sibling(const void *entry)
{
	return IS_ENABLED(CONFIG_RADIX_TREE_MULTIORDER) &&
		xa_is_internal(entry) &&
		(entry < xa_mk_sibling(XA_CHUNK_SIZE - 1));
}

#define XA_RETRY_ENTRY		xa_mk_internal(256)

/**
 * xa_is_retry() - Is the entry a retry entry?
 * @entry: Entry retrieved from the XArray
 *
 * Return: %true if the entry is a retry entry.
 */
static inline bool xa_is_retry(const void *entry)
{
	return unlikely(entry == XA_RETRY_ENTRY);
}

/**
 * typedef xa_update_node_t - A callback function from the XArray.
 * @node: The node which is being processed
 *
 * This function is called every time the XArray updates the count of
 * present and value entries in a node.  It allows advanced users to
 * maintain the private_list in the node.
 *
 * Context: The xa_lock is held and interrupts may be disabled.
 *	    Implementations should not drop the xa_lock, nor re-enable
 *	    interrupts.
 */
typedef void (*xa_update_node_t)(struct xa_node *node);

/*
 * The xa_state is opaque to its users.  It contains various different pieces
 * of state involved in the current operation on the XArray.  It should be
 * declared on the stack and passed between the various internal routines.
 * The various elements in it should not be accessed directly, but only
 * through the provided accessor functions.  The below documentation is for
 * the benefit of those working on the code, not for users of the XArray.
 *
 * @xa_node usually points to the xa_node containing the slot we're operating
 * on (and @xa_offset is the offset in the slots array).  If there is a
 * single entry in the array at index 0, there are no allocated xa_nodes to
 * point to, and so we store %NULL in @xa_node.  @xa_node is set to
 * the value %XAS_RESTART if the xa_state is not walked to the correct
 * position in the tree of nodes for this operation.  If an error occurs
 * during an operation, it is set to an %XAS_ERROR value.  If we run off the
 * end of the allocated nodes, it is set to %XAS_BOUNDS.
 */
struct xa_state {
	struct xarray *xa;
	unsigned long xa_index;
	unsigned char xa_shift;
	unsigned char xa_sibs;
	unsigned char xa_offset;
	unsigned char xa_pad;		/* Helps gcc generate better code */
	struct xa_node *xa_node;
	struct xa_node *xa_alloc;
	xa_update_node_t xa_update;
};

/*
 * We encode errnos in the xas->xa_node.  If an error has happened, we need to
 * drop the lock to fix it, and once we've done so the xa_state is invalid.
 */
#define XA_ERROR(errno) ((struct xa_node *)(((unsigned long)errno << 2) | 2UL))
#define XAS_BOUNDS	((struct xa_node *)1UL)
#define XAS_RESTART	((struct xa_node *)3UL)

#define __XA_STATE(array, index, shift, sibs)  {	\
	.xa = array,					\
	.xa_index = index,				\
	.xa_shift = shift,				\
	.xa_sibs = sibs,				\
	.xa_offset = 0,					\
	.xa_pad = 0,					\
	.xa_node = XAS_RESTART,				\
	.xa_alloc = NULL,				\
	.xa_update = NULL				\
}

/**
 * XA_STATE() - Declare an XArray operation state.
 * @name: Name of this operation state (usually xas).
 * @array: Array to operate on.
 * @index: Initial index of interest.
 *
 * Declare and initialise an xa_state on the stack.
 */
#define XA_STATE(name, array, index)				\
	struct xa_state name = __XA_STATE(array, index, 0, 0)

/**
 * XA_STATE_ORDER() - Declare an XArray operation state.
 * @name: Name of this operation state (usually xas).
 * @array: Array to operate on.
 * @index: Initial index of interest.
 * @order: Order of entry.
 *
 * Declare and initialise an xa_state on the stack.  This variant of
 * XA_STATE() allows you to specify the 'order' of the element you
 * want to operate on.`
 */
#define XA_STATE_ORDER(name, array, index, order)		\
	struct xa_state name = __XA_STATE(array,		\
			(index >> order) << order,		\
			order - (order % XA_CHUNK_SHIFT),	\
			(1U << (order % XA_CHUNK_SHIFT)) - 1)

#define xas_tagged(xas, tag)	xa_tagged((xas)->xa, (tag))
#define xas_trylock(xas)	xa_trylock((xas)->xa)
#define xas_lock(xas)		xa_lock((xas)->xa)
#define xas_unlock(xas)		xa_unlock((xas)->xa)
#define xas_lock_bh(xas)	xa_lock_bh((xas)->xa)
#define xas_unlock_bh(xas)	xa_unlock_bh((xas)->xa)
#define xas_lock_irq(xas)	xa_lock_irq((xas)->xa)
#define xas_unlock_irq(xas)	xa_unlock_irq((xas)->xa)
#define xas_lock_irqsave(xas, flags) \
				xa_lock_irqsave((xas)->xa, flags)
#define xas_unlock_irqrestore(xas, flags) \
				xa_unlock_irqrestore((xas)->xa, flags)

/**
 * xas_error() - Return an errno stored in the xa_state.
 * @xas: XArray operation state.
 *
 * Return: 0 if no error has been noted.  A negative errno if one has.
 */
static inline int xas_error(const struct xa_state *xas)
{
	return xa_err(xas->xa_node);
}

/**
 * xas_set_err() - Note an error in the xa_state.
 * @xas: XArray operation state.
 * @err: Negative error number.
 *
 * Only call this function with a negative @err; zero or positive errors
 * will probably not behave the way you think they should.  If you want
 * to clear the error from an xa_state, use xas_reset().
 */
static inline void xas_set_err(struct xa_state *xas, long err)
{
	xas->xa_node = XA_ERROR(err);
}

/**
 * xas_invalid() - Is the xas in a retry or error state?
 * @xas: XArray operation state.
 *
 * Return: %true if the xas cannot be used for operations.
 */
static inline bool xas_invalid(const struct xa_state *xas)
{
	return (unsigned long)xas->xa_node & 3;
}

/**
 * xas_valid() - Is the xas a valid cursor into the array?
 * @xas: XArray operation state.
 *
 * Return: %true if the xas can be used for operations.
 */
static inline bool xas_valid(const struct xa_state *xas)
{
	return !xas_invalid(xas);
}

/* True if the pointer is something other than a node */
static inline bool xas_not_node(struct xa_node *node)
{
	return ((unsigned long)node & 3) || !node;
}

/* True if the node represents RESTART or an error */
static inline bool xas_frozen(struct xa_node *node)
{
	return (unsigned long)node & 2;
}

/* True if the node represents head-of-tree, RESTART or BOUNDS */
static inline bool xas_top(struct xa_node *node)
{
	return node <= XAS_RESTART;
}

/**
 * xas_reset() - Reset an XArray operation state.
 * @xas: XArray operation state.
 *
 * Resets the error or walk state of the @xas so future walks of the
 * array will start from the root.  Use this if you have dropped the
 * xarray lock and want to reuse the xa_state.
 *
 * Context: Any context.
 */
static inline void xas_reset(struct xa_state *xas)
{
	xas->xa_node = XAS_RESTART;
}

/**
 * xas_retry() - Handle a retry entry.
 * @xas: XArray operation state.
 * @entry: Entry from xarray.
 *
 * An RCU-protected read may see a retry entry as a side-effect of a
 * simultaneous modification.  This function sets up the @xas to retry
 * the walk from the head of the array.
 *
 * Context: Any context.
 * Return: true if the operation needs to be retried.
 */
static inline bool xas_retry(struct xa_state *xas, const void *entry)
{
	if (!xa_is_retry(entry))
		return false;
	xas_reset(xas);
	return true;
}

void *xas_load(struct xa_state *);
void *xas_store(struct xa_state *, void *entry);
void *xas_find(struct xa_state *, unsigned long max);
void *xas_find_conflict(struct xa_state *);

bool xas_get_tag(const struct xa_state *, xa_tag_t);
void xas_set_tag(const struct xa_state *, xa_tag_t);
void xas_clear_tag(const struct xa_state *, xa_tag_t);
void *xas_find_tagged(struct xa_state *, unsigned long max, xa_tag_t);
void xas_init_tags(const struct xa_state *);

bool xas_nomem(struct xa_state *, gfp_t);
void xas_pause(struct xa_state *);

void xas_create_range(struct xa_state *);

/**
 * xas_reload() - Refetch an entry from the xarray.
 * @xas: XArray operation state.
 *
 * Use this function to check that a previously loaded entry still has
 * the same value.  This is useful for the lockless pagecache lookup where
 * we walk the array with only the RCU lock to protect us, lock the page,
 * then check that the page hasn't moved since we looked it up.
 *
 * The caller guarantees that @xas is still valid.  If it may be in an
 * error or restart state, call xas_load() instead.
 *
 * Return: The entry at this location in the xarray.
 */
static inline void *xas_reload(struct xa_state *xas)
{
	struct xa_node *node = xas->xa_node;

	if (node)
		return xa_entry(xas->xa, node, xas->xa_offset);
	return xa_head(xas->xa);
}

/**
 * xas_set() - Set up XArray operation state for a different index.
 * @xas: XArray operation state.
 * @index: New index into the XArray.
 *
 * Move the operation state to refer to a different index.  This will
 * have the effect of starting a walk from the top; see xas_next()
 * to move to an adjacent index.
 */
static inline void xas_set(struct xa_state *xas, unsigned long index)
{
	xas->xa_index = index;
	xas->xa_node = XAS_RESTART;
}

/**
 * xas_set_order() - Set up XArray operation state for a multislot entry.
 * @xas: XArray operation state.
 * @index: Target of the operation.
 * @order: Entry occupies 2^@order indices.
 */
static inline void xas_set_order(struct xa_state *xas, unsigned long index,
					unsigned int order)
{
#ifdef CONFIG_RADIX_TREE_MULTIORDER
	xas->xa_index = order < BITS_PER_LONG ? (index >> order) << order : 0;
	xas->xa_shift = order - (order % XA_CHUNK_SHIFT);
	xas->xa_sibs = (1 << (order % XA_CHUNK_SHIFT)) - 1;
	xas->xa_node = XAS_RESTART;
#else
	BUG_ON(order > 0);
	xas_set(xas, index);
#endif
}

/**
 * xas_set_update() - Set up XArray operation state for a callback.
 * @xas: XArray operation state.
 * @update: Function to call when updating a node.
 *
 * The XArray can notify a caller after it has updated an xa_node.
 * This is advanced functionality and is only needed by the page cache.
 */
static inline void xas_set_update(struct xa_state *xas, xa_update_node_t update)
{
	xas->xa_update = update;
}

/* Skip over any of these entries when iterating */
static inline bool xa_iter_skip(const void *entry)
{
	return unlikely(!entry ||
			(xa_is_internal(entry) && entry < XA_RETRY_ENTRY));
}

/**
 * xas_next_entry() - Advance iterator to next present entry.
 * @xas: XArray operation state.
 * @max: Highest index to return.
 *
 * xas_next_entry() is an inline function to optimise xarray traversal for
 * speed.  It is equivalent to calling xas_find(), and will call xas_find()
 * for all the hard cases.
 *
 * Return: The next present entry after the one currently referred to by @xas.
 */
static inline void *xas_next_entry(struct xa_state *xas, unsigned long max)
{
	struct xa_node *node = xas->xa_node;
	void *entry;

	if (unlikely(xas_not_node(node) || node->shift))
		return xas_find(xas, max);

	do {
		if (unlikely(xas->xa_index >= max))
			return xas_find(xas, max);
		if (unlikely(xas->xa_offset == XA_CHUNK_MASK))
			return xas_find(xas, max);
		xas->xa_index++;
		xas->xa_offset++;
		entry = xa_entry(xas->xa, node, xas->xa_offset);
	} while (xa_iter_skip(entry));

	return entry;
}

/* Private */
static inline unsigned int xas_find_chunk(struct xa_state *xas, bool advance,
		xa_tag_t tag)
{
	unsigned long *addr = xas->xa_node->tags[(__force unsigned)tag];
	unsigned int offset = xas->xa_offset;

	if (advance)
		offset++;
	if (XA_CHUNK_SIZE == BITS_PER_LONG) {
		if (offset < XA_CHUNK_SIZE) {
			unsigned long data = *addr & (~0UL << offset);
			if (data)
				return __ffs(data);
		}
		return XA_CHUNK_SIZE;
	}

	return find_next_bit(addr, XA_CHUNK_SIZE, offset);
}

/**
 * xas_next_tagged() - Advance iterator to next tagged entry.
 * @xas: XArray operation state.
 * @max: Highest index to return.
 * @tag: Tag to search for.
 *
 * xas_next_tagged() is an inline function to optimise xarray traversal for
 * speed.  It is equivalent to calling xas_find_tagged(), and will call
 * xas_find_tagged() for all the hard cases.
 *
 * Return: The next tagged entry after the one currently referred to by @xas.
 */
static inline void *xas_next_tagged(struct xa_state *xas, unsigned long max,
								xa_tag_t tag)
{
	struct xa_node *node = xas->xa_node;
	unsigned int offset;

	if (unlikely(xas_not_node(node) || node->shift))
		return xas_find_tagged(xas, max, tag);
	offset = xas_find_chunk(xas, true, tag);
	xas->xa_offset = offset;
	xas->xa_index = (xas->xa_index & ~XA_CHUNK_MASK) + offset;
	if (xas->xa_index > max)
		return NULL;
	if (offset == XA_CHUNK_SIZE)
		return xas_find_tagged(xas, max, tag);
	return xa_entry(xas->xa, node, offset);
}

/*
 * If iterating while holding a lock, drop the lock and reschedule
 * every %XA_CHECK_SCHED loops.
 */
enum {
	XA_CHECK_SCHED = 4096,
};

/**
 * xas_for_each() - Iterate over a range of an XArray.
 * @xas: XArray operation state.
 * @entry: Entry retrieved from the array.
 * @max: Maximum index to retrieve from array.
 *
 * The loop body will be executed for each entry present in the xarray
 * between the current xas position and @max.  @entry will be set to
 * the entry retrieved from the xarray.  It is safe to delete entries
 * from the array in the loop body.  You should hold either the RCU lock
 * or the xa_lock while iterating.  If you need to drop the lock, call
 * xas_pause() first.
 */
#define xas_for_each(xas, entry, max) \
	for (entry = xas_find(xas, max); entry; \
	     entry = xas_next_entry(xas, max))

/**
 * xas_for_each_tagged() - Iterate over a range of an XArray.
 * @xas: XArray operation state.
 * @entry: Entry retrieved from the array.
 * @max: Maximum index to retrieve from array.
 * @tag: Tag to search for.
 *
 * The loop body will be executed for each tagged entry in the xarray
 * between the current xas position and @max.  @entry will be set to
 * the entry retrieved from the xarray.  It is safe to delete entries
 * from the array in the loop body.  You should hold either the RCU lock
 * or the xa_lock while iterating.  If you need to drop the lock, call
 * xas_pause() first.
 */
#define xas_for_each_tagged(xas, entry, max, tag) \
	for (entry = xas_find_tagged(xas, max, tag); entry; \
	     entry = xas_next_tagged(xas, max, tag))

/**
 * xas_for_each_conflict() - Iterate over a range of an XArray.
 * @xas: XArray operation state.
 * @entry: Entry retrieved from the array.
 *
 * The loop body will be executed for each entry in the XArray that lies
 * within the range specified by @xas.  If the loop completes successfully,
 * any entries that lie in this range will be replaced by @entry.  The caller
 * may break out of the loop; if they do so, the contents of the XArray will
 * be unchanged.  The operation may fail due to an out of memory condition.
 * The caller may also call xa_set_err() to exit the loop while setting an
 * error to record the reason.
 */
#define xas_for_each_conflict(xas, entry) \
	while ((entry = xas_find_conflict(xas)))

void *__xas_next(struct xa_state *);
void *__xas_prev(struct xa_state *);

/**
 * xas_prev() - Move iterator to previous index.
 * @xas: XArray operation state.
 *
 * If the @xas was in an error state, it will remain in an error state
 * and this function will return %NULL.  If the @xas has never been walked,
 * it will have the effect of calling xas_load().  Otherwise one will be
 * subtracted from the index and the state will be walked to the correct
 * location in the array for the next operation.
 *
 * If the iterator was referencing index 0, this function wraps
 * around to %ULONG_MAX.
 *
 * Return: The entry at the new index.  This may be %NULL or an internal
 * entry.
 */
static inline void *xas_prev(struct xa_state *xas)
{
	struct xa_node *node = xas->xa_node;

	if (unlikely(xas_not_node(node) || node->shift ||
				xas->xa_offset == 0))
		return __xas_prev(xas);

	xas->xa_index--;
	xas->xa_offset--;
	return xa_entry(xas->xa, node, xas->xa_offset);
}

/**
 * xas_next() - Move state to next index.
 * @xas: XArray operation state.
 *
 * If the @xas was in an error state, it will remain in an error state
 * and this function will return %NULL.  If the @xas has never been walked,
 * it will have the effect of calling xas_load().  Otherwise one will be
 * added to the index and the state will be walked to the correct
 * location in the array for the next operation.
 *
 * If the iterator was referencing index %ULONG_MAX, this function wraps
 * around to 0.
 *
 * Return: The entry at the new index.  This may be %NULL or an internal
 * entry.
 */
static inline void *xas_next(struct xa_state *xas)
{
	struct xa_node *node = xas->xa_node;

	if (unlikely(xas_not_node(node) || node->shift ||
				xas->xa_offset == XA_CHUNK_MASK))
		return __xas_next(xas);

	xas->xa_index++;
	xas->xa_offset++;
	return xa_entry(xas->xa, node, xas->xa_offset);
}

/* Internal functions, mostly shared between radix-tree.c, xarray.c and idr.c */
void xas_destroy(struct xa_state *);

#endif /* _LINUX_XARRAY_H */
