// SPDX-License-Identifier: GPL-2.0+
/*
 * XArray implementation
 * Copyright (c) 2017 Microsoft Corporation
 * Author: Matthew Wilcox <willy@infradead.org>
 */

#include <linux/bitmap.h>
#include <linux/export.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/xarray.h>

/*
 * Coding conventions in this file:
 *
 * @xa is used to refer to the entire xarray.
 * @xas is the 'xarray operation state'.  It may be either a pointer to
 * an xa_state, or an xa_state stored on the stack.  This is an unfortunate
 * ambiguity.
 * @index is the index of the entry being operated on
 * @tag is an xa_tag_t; a small number indicating one of the tag bits.
 * @node refers to an xa_node; usually the primary one being operated on by
 * this function.
 * @offset is the index into the slots array inside an xa_node.
 * @parent refers to the @xa_node closer to the head than @node.
 * @entry refers to something stored in a slot in the xarray
 */

static inline unsigned int xa_lock_type(const struct xarray *xa)
{
	return (__force unsigned int)xa->xa_flags & 3;
}

static inline void xa_tag_set(struct xarray *xa, xa_tag_t tag)
{
	if (!(xa->xa_flags & XA_FLAGS_TAG(tag)))
		xa->xa_flags |= XA_FLAGS_TAG(tag);
}

static inline void xa_tag_clear(struct xarray *xa, xa_tag_t tag)
{
	if (xa->xa_flags & XA_FLAGS_TAG(tag))
		xa->xa_flags &= ~(XA_FLAGS_TAG(tag));
}

static inline bool node_get_tag(const struct xa_node *node, unsigned int offset,
				xa_tag_t tag)
{
	return test_bit(offset, node->tags[(__force unsigned)tag]);
}

/* returns true if the bit was set */
static inline bool node_set_tag(struct xa_node *node, unsigned int offset,
				xa_tag_t tag)
{
	return __test_and_set_bit(offset, node->tags[(__force unsigned)tag]);
}

/* returns true if the bit was set */
static inline bool node_clear_tag(struct xa_node *node, unsigned int offset,
				xa_tag_t tag)
{
	return __test_and_clear_bit(offset, node->tags[(__force unsigned)tag]);
}

static inline bool node_any_tag(struct xa_node *node, xa_tag_t tag)
{
	return !bitmap_empty(node->tags[(__force unsigned)tag], XA_CHUNK_SIZE);
}

#define tag_inc(tag) do { \
	tag = (__force xa_tag_t)((__force unsigned)(tag) + 1); \
} while (0)

/*
 * xas_squash_tags() - Merge all tags to the first entry
 * @xas: Array operation state.
 *
 * Set a tag on the first entry if any entry has it set.  Clear tags on
 * all sibling entries.
 */
static void xas_squash_tags(const struct xa_state *xas)
{
	unsigned int tag = 0;
	unsigned int limit = xas->xa_offset + xas->xa_sibs;

	if (!xas->xa_sibs)
		return;

	do {
		unsigned long *tags = xas->xa_node->tags[tag];
		if (find_next_bit(tags, limit, xas->xa_offset + 1) == limit)
			continue;
		__set_bit(xas->xa_offset, tags);
		bitmap_clear(tags, xas->xa_offset + 1, xas->xa_sibs);
	} while (tag++ != (__force unsigned)XA_TAG_MAX);
}

/* extracts the offset within this node from the index */
static unsigned int get_offset(unsigned long index, struct xa_node *node)
{
	return (index >> node->shift) & XA_CHUNK_MASK;
}

static void xas_set_offset(struct xa_state *xas)
{
	xas->xa_offset = get_offset(xas->xa_index, xas->xa_node);
}

/* move the index either forwards (find) or backwards (sibling slot) */
static void xas_move_index(struct xa_state *xas, unsigned long offset)
{
	unsigned int shift = xas->xa_node->shift;
	xas->xa_index &= ~XA_CHUNK_MASK << shift;
	xas->xa_index += offset << shift;
}

static void xas_advance(struct xa_state *xas)
{
	xas->xa_offset++;
	xas_move_index(xas, xas->xa_offset);
}

static void *set_bounds(struct xa_state *xas)
{
	xas->xa_node = XAS_BOUNDS;
	return NULL;
}

/*
 * Starts a walk.  If the @xas is already valid, we assume that it's on
 * the right path and just return where we've got to.  If we're in an
 * error state, return NULL.  If the index is outside the current scope
 * of the xarray, return NULL without changing @xas->xa_node.  Otherwise
 * set @xas->xa_node to NULL and return the current head of the array.
 */
static void *xas_start(struct xa_state *xas)
{
	void *entry;

	if (xas_valid(xas))
		return xas_reload(xas);
	if (xas_error(xas))
		return NULL;

	entry = xa_head(xas->xa);
	if (!xa_is_node(entry)) {
		if (xas->xa_index)
			return set_bounds(xas);
	} else {
		if ((xas->xa_index >> xa_to_node(entry)->shift) > XA_CHUNK_MASK)
			return set_bounds(xas);
	}

	xas->xa_node = NULL;
	return entry;
}

static void *xas_descend(struct xa_state *xas, struct xa_node *node)
{
	unsigned int offset = get_offset(xas->xa_index, node);
	void *entry = xa_entry(xas->xa, node, offset);

	xas->xa_node = node;
	if (xa_is_sibling(entry)) {
		offset = xa_to_sibling(entry);
		entry = xa_entry(xas->xa, node, offset);
		xas_move_index(xas, offset);
	}

	xas->xa_offset = offset;
	return entry;
}

/**
 * xas_load() - Load an entry from the XArray (advanced).
 * @xas: XArray operation state.
 *
 * Usually walks the @xas to the appropriate state to load the entry
 * stored at xa_index.  However, it will do nothing and return %NULL if
 * @xas is in an error state.  xas_load() will never expand the tree.
 *
 * If the xa_state is set up to operate on a multi-index entry, xas_load()
 * may return %NULL or an internal entry, even if there are entries
 * present within the range specified by @xas.
 *
 * Context: Any context.  The caller should hold the xa_lock or the RCU lock.
 * Return: Usually an entry in the XArray, but see description for exceptions.
 */
void *xas_load(struct xa_state *xas)
{
	void *entry = xas_start(xas);

	while (xa_is_node(entry)) {
		struct xa_node *node = xa_to_node(entry);

		if (xas->xa_shift > node->shift)
			break;
		entry = xas_descend(xas, node);
	}
	return entry;
}
EXPORT_SYMBOL_GPL(xas_load);

/* Move the radix tree node cache here */
extern struct kmem_cache *radix_tree_node_cachep;
extern void radix_tree_node_rcu_free(struct rcu_head *head);

#define XA_RCU_FREE	((struct xarray *)1)

static void xa_node_free(struct xa_node *node)
{
	XA_NODE_BUG_ON(node, !list_empty(&node->private_list));
	node->array = XA_RCU_FREE;
	call_rcu(&node->rcu_head, radix_tree_node_rcu_free);
}

/*
 * xas_destroy() - Free any resources allocated during the XArray operation.
 * @xas: XArray operation state.
 *
 * This function is now internal-only (and will be made static once
 * idr_preload() is removed).
 */
void xas_destroy(struct xa_state *xas)
{
	struct xa_node *node = xas->xa_alloc;

	if (!node)
		return;
	XA_NODE_BUG_ON(node, !list_empty(&node->private_list));
	kmem_cache_free(radix_tree_node_cachep, node);
	xas->xa_alloc = NULL;
}

/**
 * xas_nomem() - Allocate memory if needed.
 * @xas: XArray operation state.
 * @gfp: Memory allocation flags.
 *
 * If we need to add new nodes to the XArray, we try to allocate memory
 * with GFP_NOWAIT while holding the lock, which will usually succeed.
 * If it fails, @xas is flagged as needing memory to continue.  The caller
 * should drop the lock and call xas_nomem().  If xas_nomem() succeeds,
 * the caller should retry the operation.
 *
 * Forward progress is guaranteed as one node is allocated here and
 * stored in the xa_state where it will be found by xas_alloc().  More
 * nodes will likely be found in the slab allocator, but we do not tie
 * them up here.
 *
 * Return: true if memory was needed, and was successfully allocated.
 */
bool xas_nomem(struct xa_state *xas, gfp_t gfp)
{
	if (xas->xa_node != XA_ERROR(-ENOMEM)) {
		xas_destroy(xas);
		return false;
	}
	xas->xa_alloc = kmem_cache_alloc(radix_tree_node_cachep, gfp);
	if (!xas->xa_alloc)
		return false;
	XA_NODE_BUG_ON(xas->xa_alloc, !list_empty(&xas->xa_alloc->private_list));
	xas->xa_node = XAS_RESTART;
	return true;
}
EXPORT_SYMBOL_GPL(xas_nomem);

/*
 * __xas_nomem() - Drop locks and allocate memory if needed.
 * @xas: XArray operation state.
 * @gfp: Memory allocation flags.
 *
 * Internal variant of xas_nomem().
 *
 * Return: true if memory was needed, and was successfully allocated.
 */
static bool __xas_nomem(struct xa_state *xas, gfp_t gfp)
	__must_hold(xas->xa->xa_lock)
{
	unsigned int lock_type = xa_lock_type(xas->xa);

	if (xas->xa_node != XA_ERROR(-ENOMEM)) {
		xas_destroy(xas);
		return false;
	}
	if (gfpflags_allow_blocking(gfp)) {
		if (lock_type == XA_LOCK_IRQ)
			xas_unlock_irq(xas);
		else if (lock_type == XA_LOCK_BH)
			xas_unlock_bh(xas);
		else
			xas_unlock(xas);
		xas->xa_alloc = kmem_cache_alloc(radix_tree_node_cachep, gfp);
		if (lock_type == XA_LOCK_IRQ)
			xas_lock_irq(xas);
		else if (lock_type == XA_LOCK_BH)
			xas_lock_bh(xas);
		else
			xas_lock(xas);
	} else {
		xas->xa_alloc = kmem_cache_alloc(radix_tree_node_cachep, gfp);
	}
	if (!xas->xa_alloc)
		return false;
	XA_NODE_BUG_ON(xas->xa_alloc, !list_empty(&xas->xa_alloc->private_list));
	xas->xa_node = XAS_RESTART;
	return true;
}

static void xas_update(struct xa_state *xas, struct xa_node *node)
{
	if (xas->xa_update)
		xas->xa_update(node);
	else
		XA_NODE_BUG_ON(node, !list_empty(&node->private_list));
}

static void *xas_alloc(struct xa_state *xas, unsigned int shift)
{
	struct xa_node *parent = xas->xa_node;
	struct xa_node *node = xas->xa_alloc;

	if (xas_invalid(xas))
		return NULL;

	if (node) {
		xas->xa_alloc = NULL;
	} else {
		node = kmem_cache_alloc(radix_tree_node_cachep,
					GFP_NOWAIT | __GFP_NOWARN);
		if (!node) {
			xas_set_err(xas, -ENOMEM);
			return NULL;
		}
	}

	if (parent) {
		node->offset = xas->xa_offset;
		parent->count++;
		XA_NODE_BUG_ON(node, parent->count > XA_CHUNK_SIZE);
		xas_update(xas, parent);
	}
	XA_NODE_BUG_ON(node, shift > BITS_PER_LONG);
	XA_NODE_BUG_ON(node, !list_empty(&node->private_list));
	node->shift = shift;
	node->count = 0;
	node->nr_values = 0;
	RCU_INIT_POINTER(node->parent, xas->xa_node);
	node->array = xas->xa;

	return node;
}

/*
 * Use this to calculate the maximum index that will need to be created
 * in order to add the entry described by @xas.  Because we cannot store a
 * multiple-index entry at index 0, the calculation is a little more complex
 * than you might expect.
 */
static unsigned long xas_max(struct xa_state *xas)
{
	unsigned long max = xas->xa_index;

#ifdef CONFIG_RADIX_TREE_MULTIORDER
	if (xas->xa_shift || xas->xa_sibs) {
		unsigned long mask;
		mask = (((xas->xa_sibs + 1UL) << xas->xa_shift) - 1);
		max |= mask;
		if (mask == max)
			max++;
	}
#endif

	return max;
}

/* The maximum index that can be contained in the array without expanding it */
static unsigned long max_index(void *entry)
{
	if (!xa_is_node(entry))
		return 0;
	return (XA_CHUNK_SIZE << xa_to_node(entry)->shift) - 1;
}

static void xas_shrink(struct xa_state *xas)
{
	struct xarray *xa = xas->xa;
	struct xa_node *node = xas->xa_node;

	for (;;) {
		void *entry;

		XA_NODE_BUG_ON(node, node->count > XA_CHUNK_SIZE);
		if (node->count != 1)
			break;
		entry = xa_entry_locked(xa, node, 0);
		if (!entry)
			break;
		if (!xa_is_node(entry) && node->shift)
			break;
		xas->xa_node = XAS_BOUNDS;

		RCU_INIT_POINTER(xa->xa_head, entry);

		node->count = 0;
		node->nr_values = 0;
		if (!xa_is_node(entry))
			RCU_INIT_POINTER(node->slots[0], XA_RETRY_ENTRY);
		xas_update(xas, node);
		xa_node_free(node);
		if (!xa_is_node(entry))
			break;
		node = xa_to_node(entry);
		node->parent = NULL;
	}
}

/*
 * xas_delete_node() - Attempt to delete an xa_node
 * @xas: Array operation state.
 *
 * Attempts to delete the @xas->xa_node.  This will fail if xa->node has
 * a non-zero reference count.
 */
static void xas_delete_node(struct xa_state *xas)
{
	struct xa_node *node = xas->xa_node;

	for (;;) {
		struct xa_node *parent;

		XA_NODE_BUG_ON(node, node->count > XA_CHUNK_SIZE);
		if (node->count)
			break;

		parent = xa_parent_locked(xas->xa, node);
		xas->xa_node = parent;
		xas->xa_offset = node->offset;
		xa_node_free(node);

		if (!parent) {
			xas->xa->xa_head = NULL;
			xas->xa_node = XAS_BOUNDS;
			return;
		}

		parent->slots[xas->xa_offset] = NULL;
		parent->count--;
		XA_NODE_BUG_ON(parent, parent->count > XA_CHUNK_SIZE);
		node = parent;
		xas_update(xas, node);
	}

	if (!node->parent)
		xas_shrink(xas);
}

/**
 * xas_free_nodes() - Free this node and all nodes that it references
 * @xas: Array operation state.
 * @top: Node to free
 *
 * This node has been removed from the tree.  We must now free it and all
 * of its subnodes.  There may be RCU walkers with references into the tree,
 * so we must replace all entries with retry markers.
 */
static void xas_free_nodes(struct xa_state *xas, struct xa_node *top)
{
	unsigned int offset = 0;
	struct xa_node *node = top;

	for (;;) {
		void *entry = xa_entry_locked(xas->xa, node, offset);

		if (xa_is_node(entry)) {
			node = xa_to_node(entry);
			offset = 0;
			continue;
		}
		if (entry)
			RCU_INIT_POINTER(node->slots[offset], XA_RETRY_ENTRY);
		offset++;
		while (offset == XA_CHUNK_SIZE) {
			struct xa_node *parent;

			parent = xa_parent_locked(xas->xa, node);
			offset = node->offset + 1;
			node->count = 0;
			node->nr_values = 0;
			xas_update(xas, node);
			xa_node_free(node);
			if (node == top)
				return;
			node = parent;
		}
	}
}

/*
 * xas_expand adds nodes to the head of the tree until it has reached
 * sufficient height to be able to contain @xas->xa_index
 */
static int xas_expand(struct xa_state *xas, void *head)
{
	struct xarray *xa = xas->xa;
	struct xa_node *node = NULL;
	unsigned int shift = 0;
	unsigned long max = xas_max(xas);

	if (!head) {
		if (max == 0)
			return 0;
		while ((max >> shift) >= XA_CHUNK_SIZE)
			shift += XA_CHUNK_SHIFT;
		return shift + XA_CHUNK_SHIFT;
	} else if (xa_is_node(head)) {
		node = xa_to_node(head);
		shift = node->shift + XA_CHUNK_SHIFT;
	}
	xas->xa_node = NULL;

	while (max > max_index(head)) {
		xa_tag_t tag = 0;

		XA_NODE_BUG_ON(node, shift > BITS_PER_LONG);
		node = xas_alloc(xas, shift);
		if (!node)
			return -ENOMEM;

		node->count = 1;
		if (xa_is_value(head))
			node->nr_values = 1;
		RCU_INIT_POINTER(node->slots[0], head);

		/* Propagate the aggregated tag info to the new child */
		for (;;) {
			if (xa_tagged(xa, tag))
				node_set_tag(node, 0, tag);
			if (tag == XA_TAG_MAX)
				break;
			tag_inc(tag);
		}

		/*
		 * Now that the new node is fully initialised, we can add
		 * it to the tree
		 */
		if (xa_is_node(head)) {
			xa_to_node(head)->offset = 0;
			rcu_assign_pointer(xa_to_node(head)->parent, node);
		}
		head = xa_mk_node(node);
		rcu_assign_pointer(xa->xa_head, head);
		xas_update(xas, node);

		shift += XA_CHUNK_SHIFT;
	}

	xas->xa_node = node;
	return shift;
}

/*
 * xas_create() - Create a slot to store an entry in.
 * @xas: XArray operation state.
 *
 * Most users will not need to call this function directly, as it is called
 * by xas_store().  It is useful for doing conditional store operations
 * (see the xa_cmpxchg() implementation for an example).
 *
 * Return: If the slot already existed, returns the contents of this slot.
 * If the slot was newly created, returns NULL.  If it failed to create the
 * slot, returns NULL and indicates the error in @xas.
 */
static void *xas_create(struct xa_state *xas)
{
	struct xarray *xa = xas->xa;
	void *entry;
	void __rcu **slot;
	struct xa_node *node = xas->xa_node;
	int shift;
	unsigned int order = xas->xa_shift;

	if (xas_top(node)) {
		entry = xa_head_locked(xa);
		xas->xa_node = NULL;
		shift = xas_expand(xas, entry);
		if (shift < 0)
			return NULL;
		entry = xa_head_locked(xa);
		slot = &xa->xa_head;
	} else if (xas_error(xas)) {
		return NULL;
	} else if (node) {
		unsigned int offset = xas->xa_offset;

		shift = node->shift;
		entry = xa_entry_locked(xa, node, offset);
		slot = &node->slots[offset];
	} else {
		shift = 0;
		entry = xa_head_locked(xa);
		slot = &xa->xa_head;
	}

	while (shift > order) {
		shift -= XA_CHUNK_SHIFT;
		if (!entry) {
			node = xas_alloc(xas, shift);
			if (!node)
				break;
			rcu_assign_pointer(*slot, xa_mk_node(node));
		} else if (xa_is_node(entry)) {
			node = xa_to_node(entry);
		} else {
			break;
		}
		entry = xas_descend(xas, node);
		slot = &node->slots[xas->xa_offset];
	}

	return entry;
}

/**
 * xas_create_range() - Ensure that stores to this range will succeed
 * @xas: XArray operation state.
 *
 * Creates all of the slots in the range covered by @xas.  Sets @xas to
 * create single-index entries and positions it at the beginning of the
 * range.  This is for the benefit of users which have not yet been
 * converted to use multi-index entries.
 */
void xas_create_range(struct xa_state *xas)
{
	unsigned long index = xas->xa_index;
	unsigned char shift = xas->xa_shift;
	unsigned char sibs = xas->xa_sibs;

	xas->xa_index |= ((sibs + 1) << shift) - 1;
	xas->xa_shift = 0;
	xas->xa_sibs = 0;

	for (;;) {
		xas_create(xas);
		if (xas_error(xas))
			goto restore;
		if (xas->xa_index <= (index | XA_CHUNK_MASK))
			goto success;
		xas->xa_index -= XA_CHUNK_SIZE;

		for (;;) {
			struct xa_node *node = xas->xa_node;
			xas->xa_node = node->parent;
			xas->xa_offset = node->offset - 1;
			if (node->offset != 0)
				break;
		}
	}

restore:
	xas->xa_shift = shift;
	xas->xa_sibs = sibs;
	xas->xa_index = index;
	return;
success:
	xas->xa_index = index;
	if (xas->xa_node)
		xas_set_offset(xas);
}
EXPORT_SYMBOL_GPL(xas_create_range);

static void update_node(struct xa_state *xas, struct xa_node *node,
		int count, int values)
{
	if (!node || (!count && !values))
		return;

	node->count += count;
	node->nr_values += values;
	XA_NODE_BUG_ON(node, node->count > XA_CHUNK_SIZE);
	XA_NODE_BUG_ON(node, node->nr_values > XA_CHUNK_SIZE);
	xas_update(xas, node);
	if (count < 0)
		xas_delete_node(xas);
}

/**
 * xas_store() - Store this entry in the XArray.
 * @xas: XArray operation state.
 * @entry: New entry.
 *
 * If @xas is operating on a multi-index entry, the entry returned by this
 * function is essentially meaningless (it may be an internal entry or it
 * may be %NULL, even if there are non-NULL entries at some of the indices
 * covered by the range).  This is not a problem for any current users,
 * and can be changed if needed.
 *
 * Return: The old entry at this index.
 */
void *xas_store(struct xa_state *xas, void *entry)
{
	struct xa_node *node;
	void __rcu **slot = &xas->xa->xa_head;
	unsigned int offset, max;
	int count = 0;
	int values = 0;
	void *first, *next;
	bool value = xa_is_value(entry);

	if (entry)
		first = xas_create(xas);
	else
		first = xas_load(xas);

	if (xas_invalid(xas))
		return first;
	node = xas->xa_node;
	if (node && (xas->xa_shift < node->shift))
		xas->xa_sibs = 0;
	if ((first == entry) && !xas->xa_sibs)
		return first;

	next = first;
	offset = xas->xa_offset;
	max = xas->xa_offset + xas->xa_sibs;
	if (node) {
		slot = &node->slots[offset];
		if (xas->xa_sibs)
			xas_squash_tags(xas);
	}
	if (!entry)
		xas_init_tags(xas);

	for (;;) {
		/*
		 * Must clear the tags before setting the entry to NULL,
		 * otherwise xas_for_each_tagged may find a NULL entry and
		 * stop early.  rcu_assign_pointer contains a release barrier
		 * so the tag clearing will appear to happen before the
		 * entry is set to NULL.
		 */
		rcu_assign_pointer(*slot, entry);
		if (xa_is_node(next))
			xas_free_nodes(xas, xa_to_node(next));
		if (!node)
			break;
		count += !next - !entry;
		values += !xa_is_value(first) - !value;
		if (entry) {
			if (offset == max)
				break;
			if (!xa_is_sibling(entry))
				entry = xa_mk_sibling(xas->xa_offset);
		} else {
			if (offset == XA_CHUNK_MASK)
				break;
		}
		next = xa_entry_locked(xas->xa, node, ++offset);
		if (!xa_is_sibling(next)) {
			if (!entry && (offset > max))
				break;
			first = next;
		}
		slot++;
	}

	update_node(xas, node, count, values);
	return first;
}
EXPORT_SYMBOL_GPL(xas_store);

/**
 * xas_get_tag() - Returns the state of this tag.
 * @xas: XArray operation state.
 * @tag: Tag number.
 *
 * Return: true if the tag is set, false if the tag is clear or @xas
 * is in an error state.
 */
bool xas_get_tag(const struct xa_state *xas, xa_tag_t tag)
{
	if (xas_invalid(xas))
		return false;
	if (!xas->xa_node)
		return xa_tagged(xas->xa, tag);
	return node_get_tag(xas->xa_node, xas->xa_offset, tag);
}
EXPORT_SYMBOL_GPL(xas_get_tag);

/**
 * xas_set_tag() - Sets the tag on this entry and its parents.
 * @xas: XArray operation state.
 * @tag: Tag number.
 *
 * Sets the specified tag on this entry, and walks up the tree setting it
 * on all the ancestor entries.  Does nothing if @xas has not been walked to
 * an entry, or is in an error state.
 */
void xas_set_tag(const struct xa_state *xas, xa_tag_t tag)
{
	struct xa_node *node = xas->xa_node;
	unsigned int offset = xas->xa_offset;

	if (xas_invalid(xas))
		return;

	while (node) {
		if (node_set_tag(node, offset, tag))
			return;
		offset = node->offset;
		node = xa_parent_locked(xas->xa, node);
	}

	if (!xa_tagged(xas->xa, tag))
		xa_tag_set(xas->xa, tag);
}
EXPORT_SYMBOL_GPL(xas_set_tag);

/**
 * xas_clear_tag() - Clears the tag on this entry and its parents.
 * @xas: XArray operation state.
 * @tag: Tag number.
 *
 * Clears the specified tag on this entry, and walks back to the head
 * attempting to clear it on all the ancestor entries.  Does nothing if
 * @xas has not been walked to an entry, or is in an error state.
 */
void xas_clear_tag(const struct xa_state *xas, xa_tag_t tag)
{
	struct xa_node *node = xas->xa_node;
	unsigned int offset = xas->xa_offset;

	if (xas_invalid(xas))
		return;

	while (node) {
		if (!node_clear_tag(node, offset, tag))
			return;
		if (node_any_tag(node, tag))
			return;

		offset = node->offset;
		node = xa_parent_locked(xas->xa, node);
	}

	if (xa_tagged(xas->xa, tag))
		xa_tag_clear(xas->xa, tag);
}
EXPORT_SYMBOL_GPL(xas_clear_tag);

/**
 * xas_init_tags() - Initialise all tags for the entry
 * @xas: Array operations state.
 *
 * Initialise all tags for the entry specified by @xas.  If we're tracking
 * free entries with a tag, we need to set it on all entries.  All other
 * tags are cleared.
 *
 * This implementation is not as efficient as it could be; we may walk
 * up the tree multiple times.
 */
void xas_init_tags(const struct xa_state *xas)
{
	xa_tag_t tag = 0;

	for (;;) {
		xas_clear_tag(xas, tag);
		if (tag == XA_TAG_MAX)
			break;
		tag_inc(tag);
	}
}
EXPORT_SYMBOL_GPL(xas_init_tags);

/**
 * xas_pause() - Pause a walk to drop a lock.
 * @xas: XArray operation state.
 *
 * Some users need to pause a walk and drop the lock they're holding in
 * order to yield to a higher priority thread or carry out an operation
 * on an entry.  Those users should call this function before they drop
 * the lock.  It resets the @xas to be suitable for the next iteration
 * of the loop after the user has reacquired the lock.  If most entries
 * found during a walk require you to call xas_pause(), the xa_for_each()
 * iterator may be more appropriate.
 *
 * Note that xas_pause() only works for forward iteration.  If a user needs
 * to pause a reverse iteration, we will need a xas_pause_rev().
 */
void xas_pause(struct xa_state *xas)
{
	struct xa_node *node = xas->xa_node;

	if (xas_invalid(xas))
		return;

	if (node) {
		unsigned int offset = xas->xa_offset;
		while (++offset < XA_CHUNK_SIZE) {
			if (!xa_is_sibling(xa_entry(xas->xa, node, offset)))
				break;
		}
		xas->xa_index += (offset - xas->xa_offset) << node->shift;
	} else {
		xas->xa_index++;
	}
	xas->xa_node = XAS_RESTART;
}
EXPORT_SYMBOL_GPL(xas_pause);

/*
 * __xas_prev() - Find the previous entry in the XArray.
 * @xas: XArray operation state.
 *
 * Helper function for xas_prev() which handles all the complex cases
 * out of line.
 */
void *__xas_prev(struct xa_state *xas)
{
	void *entry;

	if (!xas_frozen(xas->xa_node))
		xas->xa_index--;
	if (xas_not_node(xas->xa_node))
		return xas_load(xas);

	if (xas->xa_offset != get_offset(xas->xa_index, xas->xa_node))
		xas->xa_offset--;

	while (xas->xa_offset == 255) {
		xas->xa_offset = xas->xa_node->offset - 1;
		xas->xa_node = xa_parent(xas->xa, xas->xa_node);
		if (!xas->xa_node)
			return set_bounds(xas);
	}

	for (;;) {
		entry = xa_entry(xas->xa, xas->xa_node, xas->xa_offset);
		if (!xa_is_node(entry))
			return entry;

		xas->xa_node = xa_to_node(entry);
		xas_set_offset(xas);
	}
}
EXPORT_SYMBOL_GPL(__xas_prev);

/*
 * __xas_next() - Find the next entry in the XArray.
 * @xas: XArray operation state.
 *
 * Helper function for xas_next() which handles all the complex cases
 * out of line.
 */
void *__xas_next(struct xa_state *xas)
{
	void *entry;

	if (!xas_frozen(xas->xa_node))
		xas->xa_index++;
	if (xas_not_node(xas->xa_node))
		return xas_load(xas);

	if (xas->xa_offset != get_offset(xas->xa_index, xas->xa_node))
		xas->xa_offset++;

	while (xas->xa_offset == XA_CHUNK_SIZE) {
		xas->xa_offset = xas->xa_node->offset + 1;
		xas->xa_node = xa_parent(xas->xa, xas->xa_node);
		if (!xas->xa_node)
			return set_bounds(xas);
	}

	for (;;) {
		entry = xa_entry(xas->xa, xas->xa_node, xas->xa_offset);
		if (!xa_is_node(entry))
			return entry;

		xas->xa_node = xa_to_node(entry);
		xas_set_offset(xas);
	}
}
EXPORT_SYMBOL_GPL(__xas_next);

/**
 * xas_find() - Find the next present entry in the XArray.
 * @xas: XArray operation state.
 * @max: Highest index to return.
 *
 * If the xas has not yet been walked to an entry, return the entry
 * which has an index >= xas.xa_index.  If it has been walked, the entry
 * currently being pointed at has been processed, and so we move to the
 * next entry.
 *
 * If no entry is found and the array is smaller than @max, the iterator
 * is set to the smallest index not yet in the array.  This allows @xas
 * to be immediately passed to xas_store().
 *
 * Return: The entry, if found, otherwise NULL.
 */
void *xas_find(struct xa_state *xas, unsigned long max)
{
	void *entry;

	if (xas_error(xas))
		return NULL;

	if (!xas->xa_node) {
		xas->xa_index = 1;
		return set_bounds(xas);
	} else if (xas_top(xas->xa_node)) {
		entry = xas_load(xas);
		if (entry || xas_not_node(xas->xa_node))
			return entry;
	}

	xas_advance(xas);

	while (xas->xa_node && (xas->xa_index <= max)) {
		if (unlikely(xas->xa_offset == XA_CHUNK_SIZE)) {
			xas->xa_offset = xas->xa_node->offset + 1;
			xas->xa_node = xa_parent(xas->xa, xas->xa_node);
			continue;
		}

		entry = xa_entry(xas->xa, xas->xa_node, xas->xa_offset);
		if (xa_is_node(entry)) {
			xas->xa_node = xa_to_node(entry);
			xas->xa_offset = 0;
			continue;
		}
		if (!xa_iter_skip(entry))
			return entry;

		xas_advance(xas);
	}

	if (!xas->xa_node)
		xas->xa_node = XAS_BOUNDS;
	return NULL;
}
EXPORT_SYMBOL_GPL(xas_find);

/**
 * xas_find_tagged() - Find the next tagged entry in the XArray.
 * @xas: XArray operation state.
 * @max: Highest index to return.
 * @tag: Tag number to search for.
 *
 * If the xas has not yet been walked to an entry, return the tagged entry
 * which has an index >= xas.xa_index.  If it has been walked, the entry
 * currently being pointed at has been processed, and so we move to the
 * next tagged entry.
 *
 * If no tagged entry is found and the array is smaller than @max, @xas is
 * set to the bounds state and xas->xa_index is set to the smallest index
 * not yet in the array.  This allows @xas to be immediately passed to
 * xas_store().
 *
 * Return: The entry, if found, otherwise %NULL.
 */
void *xas_find_tagged(struct xa_state *xas, unsigned long max, xa_tag_t tag)
{
	bool advance = true;
	unsigned int offset;
	void *entry;

	if (xas_error(xas))
		return NULL;

	if (!xas->xa_node) {
		xas->xa_index = 1;
		goto out;
	} else if (xas_top(xas->xa_node)) {
		advance = false;
		entry = xa_head(xas->xa);
		if (xas->xa_index > max_index(entry))
			goto out;
		if (!xa_is_node(entry)) {
			if (xa_tagged(xas->xa, tag)) {
				xas->xa_node = NULL;
				return entry;
			}
			xas->xa_index = 1;
			goto out;
		}
		xas->xa_node = xa_to_node(entry);
		xas->xa_offset = xas->xa_index >> xas->xa_node->shift;
	}

	while (xas->xa_index <= max) {
		if (unlikely(xas->xa_offset == XA_CHUNK_SIZE)) {
			xas->xa_offset = xas->xa_node->offset + 1;
			xas->xa_node = xa_parent(xas->xa, xas->xa_node);
			if (!xas->xa_node)
				break;
			advance = false;
			continue;
		}

		if (!advance) {
			entry = xa_entry(xas->xa, xas->xa_node, xas->xa_offset);
			if (xa_is_sibling(entry)) {
				xas->xa_offset = xa_to_sibling(entry);
				xas_move_index(xas, xas->xa_offset);
			}
		}

		offset = xas_find_chunk(xas, advance, tag);
		if (offset > xas->xa_offset) {
			advance = false;
			xas_move_index(xas, offset);
			xas->xa_offset = offset;
			if (offset == XA_CHUNK_SIZE)
				continue;
			if (xas->xa_index > max)
				break;
		}

		entry = xa_entry(xas->xa, xas->xa_node, xas->xa_offset);
		if (!xa_is_node(entry))
			return entry;
		xas->xa_node = xa_to_node(entry);
		xas_set_offset(xas);
	}

 out:
	if (!xas->xa_node)
		xas->xa_node = XAS_BOUNDS;
	return NULL;
}
EXPORT_SYMBOL_GPL(xas_find_tagged);

/**
 * xas_find_conflict() - Find the next present entry in a range.
 * @xas: XArray operation state.
 *
 * The @xas describes both a range and a position within that range.
 *
 * Context: Any context.  Expects xa_lock to be held.
 * Return: The next entry in the range covered by @xas or %NULL.
 */
void *xas_find_conflict(struct xa_state *xas)
{
	void *curr;

	if (xas_error(xas))
		return NULL;

	if (!xas->xa_node)
		return NULL;

	if (xas_top(xas->xa_node)) {
		curr = xas_start(xas);
		if (!curr)
			return NULL;
		while (xa_is_node(curr)) {
			struct xa_node *node = xa_to_node(curr);
			curr = xas_descend(xas, node);
		}
		if (curr)
			return curr;
	}

	if (xas->xa_node->shift > xas->xa_shift)
		return NULL;

	for (;;) {
		if (xas->xa_node->shift == xas->xa_shift) {
			if ((xas->xa_offset & xas->xa_sibs) == xas->xa_sibs)
				break;
		} else if (xas->xa_offset == XA_CHUNK_MASK) {
			xas->xa_offset = xas->xa_node->offset;
			xas->xa_node = xas->xa_node->parent;
			if (!xas->xa_node)
				break;
			continue;
		}
		curr = xa_entry_locked(xas->xa, xas->xa_node, ++xas->xa_offset);
		if (xa_is_sibling(curr))
			continue;
		while (xa_is_node(curr)) {
			xas->xa_node = xa_to_node(curr);
			xas->xa_offset = 0;
			curr = xa_entry_locked(xas->xa, xas->xa_node, 0);
		}
		if (curr)
			return curr;
	}
	xas->xa_offset -= xas->xa_sibs;
	return NULL;
}
EXPORT_SYMBOL_GPL(xas_find_conflict);

/**
 * xa_init_flags() - Initialise an empty XArray with flags.
 * @xa: XArray.
 * @flags: XA_FLAG values.
 *
 * If you need to initialise an XArray with special flags (eg you need
 * to take the lock from interrupt context), use this function instead
 * of xa_init().
 *
 * Context: Any context.
 */
void xa_init_flags(struct xarray *xa, gfp_t flags)
{
	unsigned int lock_type;
	static struct lock_class_key xa_lock_irq;
	static struct lock_class_key xa_lock_bh;

	spin_lock_init(&xa->xa_lock);
	xa->xa_flags = flags;
	xa->xa_head = NULL;

	lock_type = xa_lock_type(xa);
	if (lock_type == XA_LOCK_IRQ)
		lockdep_set_class(&xa->xa_lock, &xa_lock_irq);
	else if (lock_type == XA_LOCK_BH)
		lockdep_set_class(&xa->xa_lock, &xa_lock_bh);
}
EXPORT_SYMBOL(xa_init_flags);

/**
 * xa_load() - Load an entry from an XArray.
 * @xa: XArray.
 * @index: index into array.
 *
 * Context: Any context.  Takes and releases the RCU lock.
 * Return: The entry at @index in @xa.
 */
void *xa_load(struct xarray *xa, unsigned long index)
{
	XA_STATE(xas, xa, index);
	void *entry;

	rcu_read_lock();
	do {
		entry = xas_load(&xas);
	} while (xas_retry(&xas, entry));
	rcu_read_unlock();

	return entry;
}
EXPORT_SYMBOL(xa_load);

static void *xas_result(struct xa_state *xas, void *curr)
{
	XA_NODE_BUG_ON(xas->xa_node, xa_is_internal(curr));
	if (xas_error(xas))
		curr = xas->xa_node;
	return curr;
}

/**
 * __xa_erase() - Erase this entry from the XArray while locked.
 * @xa: XArray.
 * @index: Index into array.
 *
 * If the entry at this index is a multi-index entry then all indices will
 * be erased, and the entry will no longer be a multi-index entry.
 * This function expects the xa_lock to be held on entry.
 *
 * Context: Any context.  Expects xa_lock to be held on entry.  May
 * release and reacquire xa_lock if @gfp flags permit.
 * Return: The old entry at this index.
 */
void *__xa_erase(struct xarray *xa, unsigned long index)
{
	XA_STATE(xas, xa, index);
	return xas_result(&xas, xas_store(&xas, NULL));
}
EXPORT_SYMBOL_GPL(__xa_erase);

/**
 * xa_store() - Store this entry in the XArray.
 * @xa: XArray.
 * @index: Index into array.
 * @entry: New entry.
 * @gfp: Memory allocation flags.
 *
 * After this function returns, loads from this index will return @entry.
 * Storing into an existing multislot entry updates the entry of every index.
 * The tags associated with @index are unaffected unless @entry is %NULL.
 *
 * Context: Process context.  Takes and releases the xa_lock.  May sleep
 * if the @gfp flags permit.
 * Return: The old entry at this index on success, xa_err(-EINVAL) if @entry
 * cannot be stored in an XArray, or xa_err(-ENOMEM) if memory allocation
 * failed.
 */
void *xa_store(struct xarray *xa, unsigned long index, void *entry, gfp_t gfp)
{
	XA_STATE(xas, xa, index);
	void *curr;

	if (WARN_ON_ONCE(xa_is_internal(entry)))
		return XA_ERROR(-EINVAL);

	do {
		xas_lock(&xas);
		curr = xas_store(&xas, entry);
		xas_unlock(&xas);
	} while (xas_nomem(&xas, gfp));

	return xas_result(&xas, curr);
}
EXPORT_SYMBOL(xa_store);

/**
 * __xa_store() - Store this entry in the XArray.
 * @xa: XArray.
 * @index: Index into array.
 * @entry: New entry.
 * @gfp: Memory allocation flags.
 *
 * You must already be holding the xa_lock when calling this function.
 * It will drop the lock if needed to allocate memory, and then reacquire
 * it afterwards.
 *
 * Context: Any context.  Expects xa_lock to be held on entry.  May
 * release and reacquire xa_lock if @gfp flags permit.
 * Return: The old entry at this index or xa_err() if an error happened.
 */
void *__xa_store(struct xarray *xa, unsigned long index, void *entry, gfp_t gfp)
{
	XA_STATE(xas, xa, index);
	void *curr;

	if (WARN_ON_ONCE(xa_is_internal(entry)))
		return XA_ERROR(-EINVAL);

	do {
		curr = xas_store(&xas, entry);
	} while (__xas_nomem(&xas, gfp));

	return xas_result(&xas, curr);
}
EXPORT_SYMBOL(__xa_store);

/**
 * xa_cmpxchg() - Conditionally replace an entry in the XArray.
 * @xa: XArray.
 * @index: Index into array.
 * @old: Old value to test against.
 * @entry: New value to place in array.
 * @gfp: Memory allocation flags.
 *
 * If the entry at @index is the same as @old, replace it with @entry.
 * If the return value is equal to @old, then the exchange was successful.
 *
 * Context: Process context.  Takes and releases the xa_lock.  May sleep
 * if the @gfp flags permit.
 * Return: The old value at this index or xa_err() if an error happened.
 */
void *xa_cmpxchg(struct xarray *xa, unsigned long index,
			void *old, void *entry, gfp_t gfp)
{
	XA_STATE(xas, xa, index);
	void *curr;

	if (WARN_ON_ONCE(xa_is_internal(entry)))
		return XA_ERROR(-EINVAL);

	do {
		xas_lock(&xas);
		curr = xas_load(&xas);
		if (curr == old)
			xas_store(&xas, entry);
		xas_unlock(&xas);
	} while (xas_nomem(&xas, gfp));

	return xas_result(&xas, curr);
}
EXPORT_SYMBOL(xa_cmpxchg);

/**
 * __xa_cmpxchg() - Store this entry in the XArray.
 * @xa: XArray.
 * @index: Index into array.
 * @old: Old value to test against.
 * @entry: New entry.
 * @gfp: Memory allocation flags.
 *
 * You must already be holding the xa_lock when calling this function.
 * It will drop the lock if needed to allocate memory, and then reacquire
 * it afterwards.
 *
 * Context: Any context.  Expects xa_lock to be held on entry.  May
 * release and reacquire xa_lock if @gfp flags permit.
 * Return: The old entry at this index or xa_err() if an error happened.
 */
void *__xa_cmpxchg(struct xarray *xa, unsigned long index,
			void *old, void *entry, gfp_t gfp)
{
	XA_STATE(xas, xa, index);
	void *curr;

	if (WARN_ON_ONCE(xa_is_internal(entry)))
		return XA_ERROR(-EINVAL);

	do {
		curr = xas_load(&xas);
		if (curr == old)
			xas_store(&xas, entry);
	} while (__xas_nomem(&xas, gfp));

	return xas_result(&xas, curr);
}
EXPORT_SYMBOL(__xa_cmpxchg);

/**
 * __xa_set_tag() - Set this tag on this entry while locked.
 * @xa: XArray.
 * @index: Index of entry.
 * @tag: Tag number.
 *
 * Attempting to set a tag on a NULL entry does not succeed.
 *
 * Context: Any context.  Expects xa_lock to be held on entry.
 */
void __xa_set_tag(struct xarray *xa, unsigned long index, xa_tag_t tag)
{
	XA_STATE(xas, xa, index);
	void *entry = xas_load(&xas);

	if (entry)
		xas_set_tag(&xas, tag);
}
EXPORT_SYMBOL_GPL(__xa_set_tag);

/**
 * __xa_clear_tag() - Clear this tag on this entry while locked.
 * @xa: XArray.
 * @index: Index of entry.
 * @tag: Tag number.
 *
 * Context: Any context.  Expects xa_lock to be held on entry.
 */
void __xa_clear_tag(struct xarray *xa, unsigned long index, xa_tag_t tag)
{
	XA_STATE(xas, xa, index);
	void *entry = xas_load(&xas);

	if (entry)
		xas_clear_tag(&xas, tag);
}
EXPORT_SYMBOL_GPL(__xa_clear_tag);

/**
 * xa_get_tag() - Inquire whether this tag is set on this entry.
 * @xa: XArray.
 * @index: Index of entry.
 * @tag: Tag number.
 *
 * This function uses the RCU read lock, so the result may be out of date
 * by the time it returns.  If you need the result to be stable, use a lock.
 *
 * Context: Any context.  Takes and releases the RCU lock.
 * Return: True if the entry at @index has this tag set, false if it doesn't.
 */
bool xa_get_tag(struct xarray *xa, unsigned long index, xa_tag_t tag)
{
	XA_STATE(xas, xa, index);
	void *entry;

	rcu_read_lock();
	entry = xas_start(&xas);
	while (xas_get_tag(&xas, tag)) {
		if (!xa_is_node(entry))
			goto found;
		entry = xas_descend(&xas, xa_to_node(entry));
	}
	rcu_read_unlock();
	return false;
 found:
	rcu_read_unlock();
	return true;
}
EXPORT_SYMBOL(xa_get_tag);

/**
 * xa_set_tag() - Set this tag on this entry.
 * @xa: XArray.
 * @index: Index of entry.
 * @tag: Tag number.
 *
 * Attempting to set a tag on a NULL entry does not succeed.
 *
 * Context: Process context.  Takes and releases the xa_lock.
 */
void xa_set_tag(struct xarray *xa, unsigned long index, xa_tag_t tag)
{
	xa_lock(xa);
	__xa_set_tag(xa, index, tag);
	xa_unlock(xa);
}
EXPORT_SYMBOL(xa_set_tag);

/**
 * xa_clear_tag() - Clear this tag on this entry.
 * @xa: XArray.
 * @index: Index of entry.
 * @tag: Tag number.
 *
 * Clearing a tag always succeeds.
 *
 * Context: Process context.  Takes and releases the xa_lock.
 */
void xa_clear_tag(struct xarray *xa, unsigned long index, xa_tag_t tag)
{
	xa_lock(xa);
	__xa_clear_tag(xa, index, tag);
	xa_unlock(xa);
}
EXPORT_SYMBOL(xa_clear_tag);

/**
 * xa_find() - Search the XArray for an entry.
 * @xa: XArray.
 * @indexp: Pointer to an index.
 * @max: Maximum index to search to.
 * @filter: Selection criterion.
 *
 * Finds the entry in @xa which matches the @filter, and has the lowest
 * index that is at least @indexp and no more than @max.
 * If an entry is found, @indexp is updated to be the index of the entry.
 * This function is protected by the RCU read lock, so it may not find
 * entries which are being simultaneously added.  It will not return an
 * %XA_RETRY_ENTRY; if you need to see retry entries, use xas_find().
 *
 * Context: Any context.  Takes and releases the RCU lock.
 * Return: The entry, if found, otherwise NULL.
 */
void *xa_find(struct xarray *xa, unsigned long *indexp,
			unsigned long max, xa_tag_t filter)
{
	XA_STATE(xas, xa, *indexp);
	void *entry;

	rcu_read_lock();
	do {
		if ((__force unsigned int)filter < XA_MAX_TAGS)
			entry = xas_find_tagged(&xas, max, filter);
		else
			entry = xas_find(&xas, max);
	} while (xas_retry(&xas, entry));
	rcu_read_unlock();

	if (entry)
		*indexp = xas.xa_index;
	return entry;
}
EXPORT_SYMBOL(xa_find);

/**
 * xa_find_after() - Search the XArray for a present entry.
 * @xa: XArray.
 * @indexp: Pointer to an index.
 * @max: Maximum index to search to.
 * @filter: Selection criterion.
 *
 * Finds the entry in @xa which matches the @filter and has the lowest
 * index that is above @indexp and no more than @max.
 * If an entry is found, @indexp is updated to be the index of the entry.
 * This function is protected by the RCU read lock, so it may miss entries
 * which are being simultaneously added.  It will not return an
 * %XA_RETRY_ENTRY; if you need to see retry entries, use xas_find().
 *
 * Context: Any context.  Takes and releases the RCU lock.
 * Return: The pointer, if found, otherwise NULL.
 */
void *xa_find_after(struct xarray *xa, unsigned long *indexp,
			unsigned long max, xa_tag_t filter)
{
	XA_STATE(xas, xa, *indexp + 1);
	void *entry;

	rcu_read_lock();
	do {
		if ((__force unsigned int)filter < XA_MAX_TAGS)
			entry = xas_find_tagged(&xas, max, filter);
		else
			entry = xas_find(&xas, max);
		if (*indexp >= xas.xa_index)
			entry = xas_next_entry(&xas, max);
	} while (xas_retry(&xas, entry));
	rcu_read_unlock();

	if (entry)
		*indexp = xas.xa_index;
	return entry;
}
EXPORT_SYMBOL(xa_find_after);

static unsigned int xas_extract_present(struct xa_state *xas, void **dst,
			unsigned long max, unsigned int n)
{
	void *entry;
	unsigned int i = 0;

	rcu_read_lock();
	xas_for_each(xas, entry, max) {
		if (xas_retry(xas, entry))
			continue;
		dst[i++] = entry;
		if (i == n)
			break;
	}
	rcu_read_unlock();

	return i;
}

static unsigned int xas_extract_tag(struct xa_state *xas, void **dst,
			unsigned long max, unsigned int n, xa_tag_t tag)
{
	void *entry;
	unsigned int i = 0;

	rcu_read_lock();
	xas_for_each_tagged(xas, entry, max, tag) {
		if (xas_retry(xas, entry))
			continue;
		dst[i++] = entry;
		if (i == n)
			break;
	}
	rcu_read_unlock();

	return i;
}

/**
 * xa_extract() - Copy selected entries from the XArray into a normal array.
 * @xa: The source XArray to copy from.
 * @dst: The buffer to copy entries into.
 * @start: The first index in the XArray eligible to be selected.
 * @max: The last index in the XArray eligible to be selected.
 * @n: The maximum number of entries to copy.
 * @filter: Selection criterion.
 *
 * Copies up to @n entries that match @filter from the XArray.  The
 * copied entries will have indices between @start and @max, inclusive.
 *
 * The @filter may be an XArray tag value, in which case entries which are
 * tagged with that tag will be copied.  It may also be %XA_PRESENT, in
 * which case non-NULL entries will be copied.
 *
 * The entries returned may not represent a snapshot of the XArray at a
 * moment in time.  For example, if another thread stores to index 5, then
 * index 10, calling xa_extract() may return the old contents of index 5
 * and the new contents of index 10.  Indices not modified while this
 * function is running will not be skipped.
 *
 * If you need stronger guarantees, holding the xa_lock across calls to this
 * function will prevent concurrent modification.
 *
 * Context: Any context.  Takes and releases the RCU lock.
 * Return: The number of entries copied.
 */
unsigned int xa_extract(struct xarray *xa, void **dst, unsigned long start,
			unsigned long max, unsigned int n, xa_tag_t filter)
{
	XA_STATE(xas, xa, start);

	if (!n)
		return 0;

	if ((__force unsigned int)filter < XA_MAX_TAGS)
		return xas_extract_tag(&xas, dst, max, n, filter);
	return xas_extract_present(&xas, dst, max, n);
}
EXPORT_SYMBOL(xa_extract);

/**
 * xa_destroy() - Free all internal data structures.
 * @xa: XArray.
 *
 * After calling this function, the XArray is empty and has freed all memory
 * allocated for its internal data structures.  You are responsible for
 * freeing the objects referenced by the XArray.
 *
 * Context: Any context.  Takes and releases the xa_lock, interrupt-safe.
 */
void xa_destroy(struct xarray *xa)
{
	XA_STATE(xas, xa, 0);
	unsigned long flags;
	void *entry;

	xas.xa_node = NULL;
	xas_lock_irqsave(&xas, flags);
	entry = xa_head_locked(xa);
	RCU_INIT_POINTER(xa->xa_head, NULL);
	xas_init_tags(&xas);
	/* lockdep checks we're still holding the lock in xas_free_nodes() */
	if (xa_is_node(entry))
		xas_free_nodes(&xas, xa_to_node(entry));
	xas_unlock_irqrestore(&xas, flags);
}
EXPORT_SYMBOL(xa_destroy);

#ifdef XA_DEBUG
void xa_dump_node(const struct xa_node *node)
{
	unsigned i, j;

	if (!node)
		return;
	if ((unsigned long)node & 3) {
		pr_cont("node %px\n", node);
		return;
	}

	pr_cont("node %px %s %d parent %px shift %d count %d values %d "
		"array %px list %px %px tags",
		node, node->parent ? "offset" : "max", node->offset,
		node->parent, node->shift, node->count, node->nr_values,
		node->array, node->private_list.prev, node->private_list.next);
	for (i = 0; i < XA_MAX_TAGS; i++)
		for (j = 0; j < XA_TAG_LONGS; j++)
			pr_cont(" %lx", node->tags[i][j]);
	pr_cont("\n");
}

void xa_dump_index(unsigned long index, unsigned int shift)
{
	if (!shift)
		pr_info("%lu: ", index);
	else if (shift >= BITS_PER_LONG)
		pr_info("0-%lu: ", ~0UL);
	else
		pr_info("%lu-%lu: ", index, index | ((1UL << shift) - 1));
}

void xa_dump_entry(const void *entry, unsigned long index, unsigned long shift)
{
	if (!entry)
		return;

	xa_dump_index(index, shift);

	if (xa_is_node(entry)) {
		if (shift == 0) {
			pr_cont("%px\n", entry);
		} else {
			unsigned long i;
			struct xa_node *node = xa_to_node(entry);
			xa_dump_node(node);
			for (i = 0; i < XA_CHUNK_SIZE; i++)
				xa_dump_entry(node->slots[i],
				      index + (i << node->shift), node->shift);
		}
	} else if (xa_is_value(entry))
		pr_cont("value %ld (0x%lx) [%px]\n", xa_to_value(entry),
						xa_to_value(entry), entry);
	else if (!xa_is_internal(entry))
		pr_cont("%px\n", entry);
	else if (xa_is_retry(entry))
		pr_cont("retry (%ld)\n", xa_to_internal(entry));
	else if (xa_is_sibling(entry))
		pr_cont("sibling (slot %ld)\n", xa_to_sibling(entry));
	else
		pr_cont("UNKNOWN ENTRY (%px)\n", entry);
}

void xa_dump(const struct xarray *xa)
{
	void *entry = xa->xa_head;
	unsigned int shift = 0;

	pr_info("xarray: %px head %px flags %x tags %d %d %d\n", xa, entry,
			xa->xa_flags, xa_tagged(xa, XA_TAG_0),
			xa_tagged(xa, XA_TAG_1), xa_tagged(xa, XA_TAG_2));
	if (xa_is_node(entry))
		shift = xa_to_node(entry)->shift + XA_CHUNK_SHIFT;
	xa_dump_entry(entry, 0, shift);
}
#endif
