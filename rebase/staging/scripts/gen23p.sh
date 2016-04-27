#!/bin/bash
cd /home/wrlinux6012/WindRiver/workspaceFxcljb/jwork/rebase/staging
echo Removing current repo..
rm -rf rcpl23_git
mkdir rcpl23_git
cd rcpl23_git

echo Creating base0..
#base0
cp -R ../../reference/merged_base0/linux/* .
cp -R ../rebase_patches/23base0/* .
git init
git add -A
git commit -m "rcpl23 base0"

echo Creating base1..
#base1
cp -R ../rebase_patches/23base1/* .
git add -A
git commit -m "P1_0001-S72-common-base_rebase"

echo Creating base2..
#base2
cp -R ../rebase_patches/23base2/* .
git add -A
git commit -m "P3_0001-S72-standard-base_rebase"

echo Creating base3..
#base3
cp -R ../rebase_patches/23base3/* .
git add -A
git commit -m "P4_0002-fix-build-error_rebase"

echo Creating base4..
#base4
cp -R ../rebase_patches/23base4/* .
git add -A
git commit -m "P5_0003-New-build-error_rebase"

echo Creating base5..
#base5
cp -R ../rebase_patches/23base5/* .
git add -A
git commit -m "P6_0004-implement-of_i2c_rebase"

echo Creating base6..
#base6
cp -R ../rebase_patches/23base6/* .
git add -A
git commit -m "P8_0005-workaround-kmap_rebase"

echo Creating base7..
#base7
cp -R ../rebase_patches/23base7/* .
git add -A
git commit -m "P10_0008-implement-f_taiki_rebase"

echo Creating base8..
#base8
cp -R ../rebase_patches/23base8/* .
git add -A
git commit -m "P11_0009-for_clipper_dtsi_rebase"

echo Creating base9..
#base9
cp -R ../rebase_patches/23base9/* .
git add -A
git commit -m "P12_0010-clipper_dts_rebase"

echo Creating base10..
#base10
cp -R ../rebase_patches/23base10/* .
git add -A
git commit -m "P13_0011-f_taiki-without-cmds_rebase"

echo Creating base11..
#base11
cp -R ../rebase_patches/23base11/* .
git add -A
git commit -m "P14_0012-taiki_wr_rebase"

echo Creating base12..
#base12
cp -R ../rebase_patches/23base12/* .
git rm arch/arm/configs/fujitsu_defconfig
git add -A
git commit -m "P15_0013-clipper_beta_diff_rebase"

echo Creating base13..
#base13
cp -R ../rebase_patches/23base13/* .
git add -A
git commit -m "P17_0015-enable_pcie_rebase"

echo Creating base14..
#base14
cp -R ../rebase_patches/23base14/* .
git add -A
git commit -m "P18_0016-f_taiki_builtin_rebase"

echo Creating base15..
#base15
cp -R ../rebase_patches/23base15/* .
git add -A
git commit -m "P19_0017-enable_usb_sata_rebase"

echo Creating base16..
#base16
cp -R ../rebase_patches/23base16/* .
git add -A
git commit -m "P20_0018-i2c_slave_disable_rebase"

echo Creating base17..
#base17
cp -R ../rebase_patches/23base17/* .
git add -A
git commit -m "P21_0019-fix_bootargsi_rebase"

echo Creating base18..
#base18
cp -R ../rebase_patches/23base18/* .
git add -A
git commit -m "P22_0020-PCR3_alpha_rebase"

echo Creating base19..
#base19
cp -R ../rebase_patches/23base19/* .
git add -A
git commit -m "P23_0021-PCR3_alpha2_rebase"

echo Creating base20..
#base20
cp -R ../rebase_patches/23base20/* .
git add -A
git commit -m "P24_0022-PCR3_rebase"

echo Creating base21..
#base21
cp -R ../rebase_patches/23base21/* .
git add -A
git commit -m "P25_0023-PCR3_SPI_Read_Change_rebase"

echo Creating base22..
#base22
cp -R ../rebase_patches/23base22/* .
git add -A
git commit -m "P26_0024-scb_mhu_Change_Thread_to_Tasklet_rebase"

echo Creating base23..
#base23
cp -R ../rebase_patches/23base23/* .
git add -A
git commit -m "P27_0025-PCI_MSI_Changei_rebase"

echo Creating base24..
#base24
cp -R ../rebase_patches/23base24/* .
git add -A
git commit -m "P28_0026-mailbox_SpinLock_Change_rebase"

echo Creating base25..
#base25
cp -R ../rebase_patches/23base25/* .
git add -A
git commit -m "P29_0027-sni-fixall-201511_patch_No1_rebase"

echo Creating base26..
#base26
cp -R ../rebase_patches/23base26/* .
git add -A
git commit -m "P30_0028-sni-fixall-201511_patch_No1_WRChange_rebase"

echo Creating base27..
#base27
cp -R ../rebase_patches/23base27/* .
git add -A
git commit -m "P31_0029-MemoryTypeChange_FLAT_to_SPARSE_rebase"

echo Creating base28..
#base28
cp -R ../rebase_patches/23base28/* .
git add -A
git commit -m "P32_0030-Linaro_merge_miss_rebase"

echo Creating base29..
#base29
cp -R ../rebase_patches/23base29/* .
git add -A
git commit -m "P33_0031-kernel_Chnage_81000000_to_84000000_rebase"

echo Generating patches..
#done with repo. generate patches
#git format-patch -p -4 |xargs sed -i -e '1,6d'
git format-patch -p -29

#for file in `ls *.patch`;
#do
#cat $file | head -n -3 >t1 && mv t1 $file 
#done

#copy patches to layer and rebase directories
echo Copying Patches..
cp 0001-P1_0001-S72-common-base_rebase.patch ../rebase_patches/0001-S72-common-base_rebase.patch
cp 0002-P3_0001-S72-standard-base_rebase.patch ../rebase_patches/0001-S72-standard-base_rebase.patch
cp 0003-P4_0002-fix-build-error_rebase.patch ../rebase_patches/0002-fix-build-error_rebase.patch
cp 0004-P5_0003-New-build-error_rebase.patch ../rebase_patches/0003-New-build-error_rebase.patch
cp 0005-P6_0004-implement-of_i2c_rebase.patch ../rebase_patches/0004-implement-of_i2c_rebase.patch
cp 0006-P8_0005-workaround-kmap_rebase.patch ../rebase_patches/0005-workaround-kmap_rebase.patch
cp 0007-P10_0008-implement-f_taiki_rebase.patch ../rebase_patches/0008-implement-f_taiki_rebase.patch
cp 0008-P11_0009-for_clipper_dtsi_rebase.patch ../rebase_patches/0009-for_clipper_dtsi_rebase.patch
cp 0009-P12_0010-clipper_dts_rebase.patch ../rebase_patches/0010-clipper_dts_rebase.patch
cp 0010-P13_0011-f_taiki-without-cmds_rebase.patch ../rebase_patches/0011-f_taiki-without-cmds_rebase.patch
cp 0011-P14_0012-taiki_wr_rebase.patch ../rebase_patches/0012-taiki_wr_rebase.patch
cp 0012-P15_0013-clipper_beta_diff_rebase.patch ../rebase_patches/0013-clipper_beta_diff_rebase.patch
cp 0013-P17_0015-enable_pcie_rebase.patch ../rebase_patches/0015-enable_pcie_rebase.patch
cp 0014-P18_0016-f_taiki_builtin_rebase.patch ../rebase_patches/0016-f_taiki_builtin_rebase.patch
cp 0015-P19_0017-enable_usb_sata_rebase.patch ../rebase_patches/0017-enable_usb_sata_rebase.patch
cp 0016-P20_0018-i2c_slave_disable_rebase.patch ../rebase_patches/0018-i2c_slave_disable_rebase.patch
cp 0017-P21_0019-fix_bootargsi_rebase.patch ../rebase_patches/0019-fix_bootargs_rebase.patch
cp 0018-P22_0020-PCR3_alpha_rebase.patch ../rebase_patches/0020-PCR3_alpha_rebase.patch
cp 0019-P23_0021-PCR3_alpha2_rebase.patch ../rebase_patches/0021-PCR3_alpha2_rebase.patch
cp 0020-P24_0022-PCR3_rebase.patch ../rebase_patches/0022-PCR3_rebase.patch
cp 0021-P25_0023-PCR3_SPI_Read_Change_rebase.patch ../rebase_patches/0023-PCR3_SPI_Read_Change_rebase.patch
cp 0022-P26_0024-scb_mhu_Change_Thread_to_Tasklet_rebase.patch ../rebase_patches/0024-scb_mhu_Change_Thread_to_Tasklet_rebase.patch
cp 0023-P27_0025-PCI_MSI_Changei_rebase.patch ../rebase_patches/0025-PCI_MSI_Change_rebase.patch
cp 0024-P28_0026-mailbox_SpinLock_Change_rebase.patch ../rebase_patches/0026-mailbox_SpinLock_Change_rebase.patch
cp 0025-P29_0027-sni-fixall-201511_patch_No1_rebase.patch ../rebase_patches/0027-sni-fixall-201511_patch_No1_rebase.patch
cp 0026-P30_0028-sni-fixall-201511_patch_No1_WRChange_rebase.patch ../rebase_patches/0028-sni-fixall-201511_patch_No1_WRChange_rebase.patch
cp 0027-P31_0029-MemoryTypeChange_FLAT_to_SPARSE_rebase.patch ../rebase_patches/0029-MemoryTypeChange_FLAT_to_SPARSE_rebase.patch
cp 0028-P32_0030-Linaro_merge_miss_rebase.patch ../rebase_patches/0030-Linaro_merge_miss_rebase.patch
cp 0029-P33_0031-kernel_Chnage_81000000_to_84000000_rebase.patch ../rebase_patches/0031-kernel_Chnage_81000000_to_84000000_rebase.patch

cp 0001-P1_0001-S72-common-base_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0001-S72-common-base.patch
cp 0002-P3_0001-S72-standard-base_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0001-S72-standard-base.patch
cp 0003-P4_0002-fix-build-error_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0002-fix-build-error.patch
cp 0004-P5_0003-New-build-error_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0003-New-build-error.patch
cp 0005-P6_0004-implement-of_i2c_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0004-implement-of_i2c.patch
cp 0006-P8_0005-workaround-kmap_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0005-workaround-kmap.patch
cp 0007-P10_0008-implement-f_taiki_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0008-implement-f_taiki.patch
cp 0008-P11_0009-for_clipper_dtsi_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0009-for_clipper_dtsi.patch
cp 0009-P12_0010-clipper_dts_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0010-clipper_dts.patch
cp 0010-P13_0011-f_taiki-without-cmds_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0011-f_taiki-without-cmds.patch
cp 0011-P14_0012-taiki_wr_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0012-taiki_wr.patch
cp 0012-P15_0013-clipper_beta_diff_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0013-clipper_beta_diff.patch
cp 0013-P17_0015-enable_pcie_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0015-enable_pcie.patch
cp 0014-P18_0016-f_taiki_builtin_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0016-f_taiki_builtin.patch
cp 0015-P19_0017-enable_usb_sata_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0017-enable_usb_sata.patch
cp 0016-P20_0018-i2c_slave_disable_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0018-i2c_slave_disable.patch
cp 0017-P21_0019-fix_bootargsi_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0019-fix_bootargs.patch
cp 0018-P22_0020-PCR3_alpha_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0020-PCR3_alpha.patch
cp 0019-P23_0021-PCR3_alpha2_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0021-PCR3_alpha2.patch
cp 0020-P24_0022-PCR3_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0022-PCR3.patch
cp 0021-P25_0023-PCR3_SPI_Read_Change_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0023-PCR3_SPI_Read_Change.patch
cp 0022-P26_0024-scb_mhu_Change_Thread_to_Tasklet_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0024-scb_mhu_Change_Thread_to_Tasklet.patch
cp 0023-P27_0025-PCI_MSI_Changei_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0025-PCI_MSI_Change.patch
cp 0024-P28_0026-mailbox_SpinLock_Change_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0026-mailbox_SpinLock_Change.patch
cp 0025-P29_0027-sni-fixall-201511_patch_No1_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0027-sni-fixall-201511_patch_No1.patch
cp 0026-P30_0028-sni-fixall-201511_patch_No1_WRChange_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0028-sni-fixall-201511_patch_No1_WRChange.patch
cp 0027-P31_0029-MemoryTypeChange_FLAT_to_SPARSE_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0029-MemoryTypeChange_FLAT_to_SPARSE.patch
cp 0028-P32_0030-Linaro_merge_miss_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0030-Linaro_merge_miss.patch
cp 0029-P33_0031-kernel_Chnage_81000000_to_84000000_rebase.patch ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0031-kernel_Chnage_81000000_to_84000000.patch

echo New patches.
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0001-S72-common-base.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0001-S72-standard-base.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0002-fix-build-error.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0003-New-build-error.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0004-implement-of_i2c.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0005-workaround-kmap.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0008-implement-f_taiki.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0009-for_clipper_dtsi.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0010-clipper_dts.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0011-f_taiki-without-cmds.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0012-taiki_wr.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0013-clipper_beta_diff.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0015-enable_pcie.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0016-f_taiki_builtin.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0017-enable_usb_sata.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0018-i2c_slave_disable.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0019-fix_bootargs.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0020-PCR3_alpha.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0021-PCR3_alpha2.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0022-PCR3.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0023-PCR3_SPI_Read_Change.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0024-scb_mhu_Change_Thread_to_Tasklet.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0025-PCI_MSI_Change.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0026-mailbox_SpinLock_Change.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0027-sni-fixall-201511_patch_No1.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0028-sni-fixall-201511_patch_No1_WRChange.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0029-MemoryTypeChange_FLAT_to_SPARSE.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0030-Linaro_merge_miss.patch
ls -l ../layer/4/fxcl-clipper-s72/recipes-kernel/linux/files/0031-kernel_Chnage_81000000_to_84000000.patch

cd ..

echo Done.
