/*
 * (C) 2001 Clemson University and The University of Chicago
 *
 * See COPYING in top-level directory.
 */

#include "protocol.h"
#include "orangefs-kernel.h"
#include "orangefs-bufmap.h"

static const char *orangefs_get_link(struct dentry *dentry, struct inode *inode,
				     struct delayed_call *done)
{
	char *target;

	if (!dentry)
		return ERR_PTR(-ECHILD);

	target =  ORANGEFS_I(dentry->d_inode)->link_target;

	gossip_debug(GOSSIP_INODE_DEBUG,
		     "%s: called on %s (target is %p)\n",
		     __func__, (char *)dentry->d_name.name, target);

	return target;
}

struct inode_operations orangefs_symlink_inode_operations = {
	.readlink = generic_readlink,
	.get_link = orangefs_get_link,
	.setattr = orangefs_setattr,
	.getattr = orangefs_getattr,
	.listxattr = orangefs_listxattr,
	.setxattr = generic_setxattr,
};
