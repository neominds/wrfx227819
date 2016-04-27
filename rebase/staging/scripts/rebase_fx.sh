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
 patch -p1 --dry-run --silent < ../../../reference/patches/set3/$patchname.patch
 if [ $? -ne 0 ];
 then
	echo Patch failed on br12. Aborting..
 	exit 1
 else
	patch -p1 < ../../../reference/patches/set3/$patchname.patch >/dev/null
 fi
 quiet_git add -A
 echo -ne "                                                                                                \r"
 echo -ne "[$count] Patching $patchname...\r"
 quiet_git commit -m "$patchname"
}

echo Removing current repo..
rm -rf staging_git
mkdir staging_git
cd staging_git

echo Creating 12_base..
cp -R ../../../reference/src/rcpl12linux/linux/* .
cp -R ../../../reference/src/rcpl12linux/delta/* .
git init
git config --global merge.tool meld
quiet_git add -A
quiet_git commit -m "rcpl12 base"
quiet_git branch br12
quiet_git branch br27

quiet_git checkout br27
cp -R ../../../reference/src/rcpl27linux/linux/* .
cp -R ../../../reference/src/rcpl27linux/delta/* .
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
done < "./../patchnames_to_rebase.txt"
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

		if [ -d "../../resolved/$lastone" ]; then 
			if [ ! -f ../../resolved/$lastone/resolve.txt ]; then
				echo You must create a resolve.txt for merge resolutions.
				echo Will not proceed without resolve.txt
				exit 1
			fi
			cp -R ../../resolved/$lastone/* .
			rm ./resolve.txt
			quiet_git add -A
        		quiet_git rebase --continue
			rebase_status=$?
		else
			rebase_status=0
			echo Could not find resolve directory "../../resolved/$lastone"
			next=`cat .git/rebase-apply/next`
			echo Rebased $((next-1)) of `cat .git/rebase-apply/last`.
			echo Starting mergetool.
			git mergetool
			echo Cannot commit resolutions from merge tool.
			echo Add resolved files to resolved directory and re-run rebase_am.
			exit 1
		fi
	fi
	[ "$rebase_status" -ne 0 ]
do :; done

#generate rebase patches
echo
echo All resolved, generating rebased-patches...
quiet_git format-patch -p -$count 
rm -f ./../../output/patches/*.patch
mv *.patch ./../../output/patches/
cd ../../output/patches
for f in *.patch; do 
	mv $f `echo $f|cut -c 6-`; 
done
cd - >/dev/null 
#ls -l ./../../output/patches/*.patch
echo
echo Completed `ls -l ./../../output/patches/*.patch|wc -l` rebased patches.

#create new layer fxcl-honolulu-lm2 
rm -f ./../../output/layers/3/fxcl-honolulu-lm2/recipes-kernel/linux/files/*.patch
cp ./../../output/patches/*.patch ./../../output/layers/3/fxcl-honolulu-lm2/recipes-kernel/linux/files/
ldir=../../output/layers/3/fxcl-honolulu-lm2/
ldirabs="$(dirname $(readlink -e $ldir))/$(basename $ldir)"
echo New layer created at $ldirabs

echo
echo Done.
exit 0
