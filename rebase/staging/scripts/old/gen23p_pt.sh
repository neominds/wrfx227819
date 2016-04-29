#!/bin/bash
cd /home/wrlinux6012/WindRiver/workspaceFxcljb/jwork/rebase/staging
echo Removing current repo..
rm -rf rcpl23_pt_git
mkdir rcpl23_pt_git
cd rcpl23_pt_git

echo Creating base0..
#base0
cp -R ../../reference/merged_pt_base0/linux/* .
#cp -R ../rebase_patches/preempt/23base0/* .
git init
git add -A
git commit -m "rcpl23 pt base0"

echo Creating base1..
#base1
cp -R ../rebase_patches/preempt/23base1/* .
git add -A
git commit -m "PT_P1_0001-S72-common-base_rebase"

echo Creating base2..
#base2
cp -R ../rebase_patches/preempt/23base2/* .
git add -A
git commit -m "PT_P2_0001-S72-preempt-base_rebase"

echo Creating base3..
#base3
cp -R ../rebase_patches/preempt/23base3/* .
git add -A
git commit -m "PT_P3_0005-workaround-kmap-preempt_rebase"

echo Creating base4..
#base4
cp -R ../rebase_patches/preempt/23base4/* .
git add -A
git commit -m "PT_P4_0006-fix-build-error_rebase"

echo Generating patches..
#done with repo. generate patches
#git format-patch -p -4 |xargs sed -i -e '1,6d'
git format-patch -p -4

#for file in `ls *.patch`;
#do
#cat $file | head -n -3 >t1 && mv t1 $file 
#done

#copy patches to layer and rebase directories
echo Copying Patches..
cp 0001-PT_P1_0001-S72-common-base_rebase.patch ../rebase_patches/preempt/0001-S72-common-base_rebase.patch
cp 0002-PT_P2_0001-S72-preempt-base_rebase.patch ../rebase_patches/preempt/0001-S72-preempt-base_rebase.patch
cp 0003-PT_P3_0005-workaround-kmap-preempt_rebase.patch ../rebase_patches/preempt/0005-workaround-kmap-preempt_rebase.patch
cp 0004-PT_P4_0006-fix-build-error_rebase.patch ../rebase_patches/preempt/0006-fix-build-error-preempt_rebase.patch 

#reuse the S72-common-base.patch from standard..
#cp 0001-PT_P1_0001-S72-common-base_rebase.patch  ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0001-S72-common-base.patch
cp 0002-PT_P2_0001-S72-preempt-base_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0001-S72-preempt-base.patch
cp 0003-PT_P3_0005-workaround-kmap-preempt_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0005-workaround-kmap-preempt.patch
cp 0004-PT_P4_0006-fix-build-error_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0006-fix-build-error-preempt.patch

echo New patches.
#ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0001-S72-common-base.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0001-S72-preempt-base.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0005-workaround-kmap-preempt.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0006-fix-build-error-preempt.patch

cd ..

echo Done.
