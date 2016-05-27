#!/bin/bash

#silence git yet report errors
quiet_git() {
    stdout=../tmp098.txt 
    stderr=../tmp099.txt

    if ! git "$@" </dev/null >$stdout 2>$stderr; then
    	cat $stdout|grep "Applying:"
        cat $stderr >&2
    	if [ -n "`grep -i conflict $stdout`" ]; then
    		grep -i conflict $stdout >../tmp090.txt
        	while read -r line
		do
  			echo $line;
		done <"../tmp090.txt";
		rm -f ../tmp090.txt
    	else
		cat $stdout
	fi
        rm -f $stdout $stderr
	return 1
    fi
    rm -f $stdout $stderr
    return 0
}


patch_12base()
{
 patchname=$1
 count=$2

 quiet_git checkout br12
 patch -p1 --dry-run --silent < $patches_dir_curr_base/$patchname.patch
 if [ $? -ne 0 ];
 then
	echo Patch failed on br12. Aborting..
 	exit 1
 else
	patch -p1 < $patches_dir_curr_base/$patchname.patch >/dev/null
 fi
 quiet_git add -A
 echo -ne "                                                                                                \r"
 echo -ne "[$count] Patching $patchname...\r"
 quiet_git commit -m "$patchname"
}

rebase_patches()
{
 echo Removing current repo..
 rm -rf staging_git_$kernel_type
 mkdir staging_git_$kernel_type
 cd staging_git_$kernel_type

 echo Creating 12_base..
 cp -R "$reference_dir_curr_base"_$kernel_type/linux/* .
 cp -R "$reference_dir_curr_base"_$kernel_type/delta/* .
 git init
 git config --global merge.tool meld
 quiet_git add -A
 quiet_git commit -m "rcpl12 base"
 quiet_git branch br12
 quiet_git branch br27

 quiet_git checkout br27
 cp -R "$reference_dir_new_base"_$kernel_type/linux/* .
 cp -R "$reference_dir_new_base"_$kernel_type/delta/* .
 quiet_git add -A
 quiet_git commit -m "rcpl27 base"
 quiet_git branch br27_1
 echo RCPL0027 branch set up. 

 # for all patches in list
 # apply to br12 branch one by one.
 count=0
 while IFS='' read -r line || [[ -n "$line" ]]; do
	patch_12base $line $count
	count=$((count+1))
 done < "./../"$patchlist_filename"_$kernel_type.txt" 

 echo 

 # rebase algo
 # do
 #{
 #    	status = try rebase
 #	if status = error
 #		try resolve conflict
 #		status = git rebase continue
 #	fi
 #}while (status != 0 )	

 # rebase br12 to br27_1
 echo Rebasing..
 quiet_git checkout br12
 quiet_git rebase br27_1
 rebase_status=$?
 while
	if [ $rebase_status -ne 0 ]; then
		#there is a conflict - try resolving
		lastone=`cat .git/rebase-apply/final-commit`
		echo Rebase failed at $lastone 
		echo Attempting to resolve..

		if [ -d "$resolve_dir/$lastone" ]; then 
			if [ ! -f $resolve_dir/$lastone/resolve.txt ]; then
				echo You must create a resolve.txt for merge resolutions.
				echo Will not proceed without resolve.txt
				exit 1
			fi
			cp -R $resolve_dir/$lastone/* .
			rm ./resolve.txt
			quiet_git add -A
        		quiet_git rebase --continue
			rebase_status=$?
		else
			rebase_status=0
			echo Could not find resolve directory "$resolve_dir/$lastone"
			next=`cat .git/rebase-apply/next`
			echo Rebased $((next-1)) of `cat .git/rebase-apply/last`.
			echo Starting mergetool.
			git mergetool
			echo Cannot commit resolutions from merge tool.
			echo Add resolved files to resolve directory and re-run rebase_fx.
			exit 1
		fi
	fi
	[ "$rebase_status" -ne 0 ]
 do :; done

 #generate rebase patches
 echo
 echo All resolved, generating rebased-patches...
 quiet_git format-patch -p -$count 
 mv *.patch ./../$output_dir/patches/
 #ls -l ./../$output_dir/patches/*.patch
 echo
 echo Completed `ls -l ./../$output_dir/patches/*.patch|wc -l` rebased patches.

 cd ..
}

# start point of script
# If you would need to change any input/output
# directories to the script, do it below
output_dir=./../output
output_layer=$output_dir/layers/6
resolve_dir=./../../resolved
patches_dir_curr_base=./../../../reference/patches/set6
reference_dir_curr_base=./../../../reference/src/rcpl12linux
reference_dir_new_base=./../../../reference/src/rcpl27linux
patchlist_filename=patchnames_to_rebase
release_dir=./../../releases

current=`pwd`
#cleanup
rm -f $output_dir/patches/*.patch
rm -f $output_layer/fxcl-honolulu-lm2/recipes-kernel/linux/files/*.patch

kernel_type=standard
echo "[-----Rebasing standard kernel-------]"
rebase_patches
cd $current

kernel_type=preempt
echo "[-----Rebasing preempt kernel-------]"
rebase_patches
cd $current

cd $output_dir/patches
for f in *.patch; do 
	mv $f `echo $f|cut -c 6-`; 
done
cd - >/dev/null 

#create new layer fxcl-honolulu-lm2 
cp $output_dir/patches/*.patch $output_layer/fxcl-honolulu-lm2/recipes-kernel/linux/files/

ldir=$output_layer/fxcl-honolulu-lm2/
ldirabs="$(dirname $(readlink -e $ldir))/$(basename $ldir)"
echo New layer created at $ldirabs

if [ "$1" == "release" ]; then
	if [ "$2" == "" ]; then
		echo Usage: $0 release \<release-string\> 
		echo e.g. $0 release R01 
		echo Error: release string not specified. Cannot create release.
		exit 1
	fi
	if [ ! -f "$release_dir/fxcl-honolulu-lm2_rcpl27_$2.tar.gz" ]; then
		reldirabs="$(dirname $(readlink -e $release_dir))/$(basename $release_dir)"
		echo Release mode: creating $reldirabs/fxcl-honolulu-lm2_rcpl27_$2.tar.gz
		tar zcf $release_dir/fxcl-honolulu-lm2_rcpl27_$2.tar.gz -C $output_layer .
		ls -l $reldirabs/fxcl-honolulu-lm2_rcpl27_$2.tar.gz
	else
		echo Specified release already exist. Cannot create release
		exit 1
	fi
fi

echo
echo Done.
exit 0
