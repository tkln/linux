/* mountpoint management
 *
 * Copyright (C) 2002 Red Hat, Inc. All Rights Reserved.
 * Written by David Howells (dhowells@redhat.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/pagemap.h>
#include <linux/mount.h>
#include <linux/namei.h>
#include <linux/gfp.h>
#include "internal.h"


static struct dentry *afs_mntpt_lookup(struct inode *dir,
				       struct dentry *dentry,
				       unsigned int flags);
static int afs_mntpt_open(struct inode *inode, struct file *file);
static void afs_mntpt_expiry_timed_out(struct work_struct *work);

const struct file_operations afs_mntpt_file_operations = {
	.open		= afs_mntpt_open,
	.llseek		= noop_llseek,
};

const struct inode_operations afs_mntpt_inode_operations = {
	.lookup		= afs_mntpt_lookup,
	.readlink	= page_readlink,
	.getattr	= afs_getattr,
	.listxattr	= afs_listxattr,
};

const struct inode_operations afs_autocell_inode_operations = {
	.getattr	= afs_getattr,
};

static LIST_HEAD(afs_vfsmounts);
static DECLARE_DELAYED_WORK(afs_mntpt_expiry_timer, afs_mntpt_expiry_timed_out);

static unsigned long afs_mntpt_expiry_timeout = 10 * 60;

static const char afs_root_volume[] = "root.cell";

/*
 * no valid lookup procedure on this sort of dir
 */
static struct dentry *afs_mntpt_lookup(struct inode *dir,
				       struct dentry *dentry,
				       unsigned int flags)
{
	_enter("%p,%p{%pd2}", dir, dentry, dentry);
	return ERR_PTR(-EREMOTE);
}

/*
 * no valid open procedure on this sort of dir
 */
static int afs_mntpt_open(struct inode *inode, struct file *file)
{
	_enter("%p,%p{%pD2}", inode, file, file);
	return -EREMOTE;
}

/*
 * Set the parameters for the proposed superblock.
 */
static int afs_mntpt_set_params(struct fs_context *fc, struct dentry *mntpt)
{
	struct afs_fs_context *ctx = fc->fs_private;
	struct afs_vnode *vnode = AFS_FS_I(d_inode(mntpt));
	struct afs_cell *cell;
	const char *p;
	int ret;

	if (test_bit(AFS_VNODE_PSEUDODIR, &vnode->flags)) {
		/* if the directory is a pseudo directory, use the d_name */
		unsigned size = mntpt->d_name.len;

		if (size < 2)
			return -ENOENT;

		p = mntpt->d_name.name;
		if (mntpt->d_name.name[0] == '.') {
			size--;
			p++;
			ctx->type = AFSVL_RWVOL;
			ctx->force = true;
		}
		if (size > AFS_MAXCELLNAME)
			return -ENAMETOOLONG;

		cell = afs_lookup_cell(ctx->net, p, size, NULL, false);
		if (IS_ERR(cell)) {
			pr_err("kAFS: unable to lookup cell '%pd'\n", mntpt);
			return PTR_ERR(cell);
		}
		afs_put_cell(ctx->net, ctx->cell);
		ctx->cell = cell;

		ctx->volname = afs_root_volume;
		ctx->volnamesz = sizeof(afs_root_volume) - 1;
	} else {
		/* read the contents of the AFS special symlink */
		struct page *page;
		loff_t size = i_size_read(d_inode(mntpt));
		char *buf;

		if (size > PAGE_SIZE - 1)
			return -EINVAL;

		page = read_mapping_page(d_inode(mntpt)->i_mapping, 0, NULL);
		if (IS_ERR(page))
			return PTR_ERR(page);

		if (PageError(page)) {
			put_page(page);
			return -EIO;
		}

		buf = kmap(page);
		ret = vfs_set_fs_source(fc, buf, size);
		kunmap(page);
		put_page(page);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/*
 * create a vfsmount to be automounted
 */
static struct vfsmount *afs_mntpt_do_automount(struct dentry *mntpt)
{
	struct fs_context *fc;
	struct vfsmount *mnt;
	int ret;

	BUG_ON(!d_inode(mntpt));

	fc = vfs_new_fs_context(&afs_fs_type, mntpt, 0,
				FS_CONTEXT_FOR_SUBMOUNT);
	if (IS_ERR(fc))
		return ERR_CAST(fc);

	ret = afs_mntpt_set_params(fc, mntpt);
	if (ret < 0)
		goto error_fc;

	ret = vfs_get_tree(fc);
	if (ret < 0)
		goto error_fc;

	mnt = vfs_create_mount(fc, 0);
	if (IS_ERR(mnt)) {
		ret = PTR_ERR(mnt);
		goto error_fc;
	}

	put_fs_context(fc);
	return mnt;

error_fc:
	put_fs_context(fc);
	return ERR_PTR(ret);
}

/*
 * handle an automount point
 */
struct vfsmount *afs_d_automount(struct path *path)
{
	struct vfsmount *newmnt;

	_enter("{%pd}", path->dentry);

	newmnt = afs_mntpt_do_automount(path->dentry);
	if (IS_ERR(newmnt))
		return newmnt;

	mntget(newmnt); /* prevent immediate expiration */
	mnt_set_expiry(newmnt, &afs_vfsmounts);
	queue_delayed_work(afs_wq, &afs_mntpt_expiry_timer,
			   afs_mntpt_expiry_timeout * HZ);
	_leave(" = %p", newmnt);
	return newmnt;
}

/*
 * handle mountpoint expiry timer going off
 */
static void afs_mntpt_expiry_timed_out(struct work_struct *work)
{
	_enter("");

	if (!list_empty(&afs_vfsmounts)) {
		mark_mounts_for_expiry(&afs_vfsmounts);
		queue_delayed_work(afs_wq, &afs_mntpt_expiry_timer,
				   afs_mntpt_expiry_timeout * HZ);
	}

	_leave("");
}

/*
 * kill the AFS mountpoint timer if it's still running
 */
void afs_mntpt_kill_timer(void)
{
	_enter("");

	ASSERT(list_empty(&afs_vfsmounts));
	cancel_delayed_work_sync(&afs_mntpt_expiry_timer);
}
