# Prerequisites

Before you begin, ensure the following are installed:

- **Git**
- **Git LFS**

---

## Step 1: Install Git LFS

If not already installed, set up Git LFS on your system:

```bash
git lfs install
Note: This is a one-time setup per user account.
```

## Step 2: Clone the Repository

Clone the repository using the Git CLI:

```bash
git clone https://github.com/tvsrr/ROS_Udacity
```

Step 3: Verify Large Files

Git LFS typically downloads large files automatically during checkout. After cloning:

Check file sizes: If files are small (~130 bytes), they are LFS pointers and not the actual content.
Manually fetch LFS files (if needed):

```bash

cd repository  # Navigate into the cloned directory
git lfs fetch --all  # Download all LFS objects
git lfs pull         # Replace pointers with actual files
```
