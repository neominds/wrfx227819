#!/bin/bash

#silence git yet report errors
quiet_git() {
    stdout=../tmp098.txt 
    stderr=../tmp099.txt

    echo GITARGS "$@"

    if ! git "$@" </dev/null >$stdout 2>$stderr; then
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
#       exit 1
	return 1
    fi

    rm -f $stdout $stderr
    return 0
}


patch_12base()
{
 patchname=$1
 checkoutbranch=$2
 newbranchname=$3

 quiet_git checkout $checkoutbranch
 patch -p1 --dry-run --silent < ../../../reference/patches/set1/$patchname.patch
 if [ $? -ne 0 ];
 then
	echo Patch failed on br12. Aborting..
 	exit 1
 else
	patch -p1 < ../../../reference/patches/set1/$patchname.patch >/dev/null
 fi
 quiet_git add -A
 echo Patching $patchname...
 quiet_git commit -m "$patchname"
 quiet_git branch $newbranchname
}

merge_to_27base()
{
#arguments - patchname, icheckoutbranch, mergebranch, newbranchname, resolvedir
patchname=$1
checkoutbranch=$2
mergebranch=$3
newbranchname=$4
resolvedir=$5

echo "Merging $patchname [$mergebranch --> $checkoutbranch ==> $newbranchname]"

#before merge, check if a conflicts directory exists fo this $mergebranch
#if one exists, create a temp branch, copy resolved files and do git add before merge
if [ -d "../../resolved/$mergebranch" ]; then 

	if [ ! -f ../../resolved/$mergebranch/resolve.txt ]; then
		echo You must create a resolve.txt for merge resolutions.
		echo Will not proceed without resolve.txt
		exit 1
	fi
	echo Copying files from resolve directory to $mergebranch ..
	quiet_git checkout $mergebranch
	quiet_git branch 12tmpb
	quiet_git checkout 12tmpb
	
	cp -R ../../resolved/$mergebranch/* .
	rm ./resolve.txt
	quiet_git add -A
 	quiet_git commit -m "$patchname with resolve"

	echo Copying files from resolve directory $checkoutbranch ..
	quiet_git checkout $checkoutbranch
	quiet_git branch 27tmpb
	quiet_git checkout 27tmpb

	cp -R ../../resolved/$mergebranch/* .
	rm ./resolve.txt
	quiet_git add -A
 	quiet_git commit -m "$patchname with resolve"

	quiet_git merge 12tmpb
	if [ $? -ne 0 ];
	then
		echo Unexpected error in merging resolved files to tmp branch. exiting..
		exit 1
	fi
	echo Auto-merging with resolved...
	quiet_git checkout $checkoutbranch
	quiet_git merge 27tmpb
	if [ $? -ne 0 ];
	then
		echo Unexpected error in merging resolved files to br27 branch. exiting..
		exit 1
	else
		quiet_git branch $newbranchname

		git branch -d 12tmpb
		git branch -d 27tmpb
		echo Rebasing $patchname complete.
		echo 
	fi
else
	echo Auto-merging...
	quiet_git checkout $checkoutbranch
	quiet_git merge $mergebranch 

	if [ $? -ne 0 ];
	then
		echo merge failed...
		git mergetool
		echo Cannot commit resolutions from merge tool.
		echo Add resolved files to respective resolved directory and re-run rebase_am.
		exit 1
	else
		quiet_git branch $newbranchname

		echo Rebasing $patchname complete.
		echo 
	fi
fi

}

echo Removing current repo..
rm -rf staging_git
mkdir staging_git
cd staging_git

echo Creating br_base..
#br_base
cp -R ../../../reference/src/rcpl12linux/linux/* .
cp -R ../../../reference/src/rcpl12linux/delta/* .
git init
git config --global merge.tool meld
quiet_git add -A
quiet_git commit -m "rcpl12 br_base"
quiet_git branch br_base
quiet_git branch br12_0
quiet_git branch br27_0

quiet_git checkout br27_0
cp -R ../../../reference/src/rcpl27linux/linux/* .
cp -R ../../../reference/src/rcpl27linux/delta/* .
quiet_git add -A
quiet_git commit -m "rcpl27 base"
quiet_git branch br27_1	#tag ?
echo RCPL0027 branch set up. 

#
# for all patches in list ....
# patch_12base()
# merge_to_27base()
#
count=0
while IFS='' read -r line || [[ -n "$line" ]]; do
    	echo "rebasing: $line"
	patch_12base $line br12_$count br12_$((count+1))
	merge_to_27base $line br27_$((count+1)) br12_$((count+1)) br27_$((count+2))
	count=$((count+1))
done < "./../patchnames_to_rebase.txt"

#generate patches - reuse gen23p.sh
echo $count Done.
