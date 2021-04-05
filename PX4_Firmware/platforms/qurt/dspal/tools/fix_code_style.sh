#!/bin/bash
for i in "$@"
do

case $i in
	--check)
	DRYRUN=true
	shift
	;;
	-p)
	shift
	PRUNE_DIRS="${1}"
	shift
	;;
	*)
	;;
esac
shift
done

for d in ${PRUNE_DIRS}; do
	PRUNE_CMD="${PRUNE_CMD} -path ./$d -prune -o"
done

TOOLSDIR=$( dirname "${BASH_SOURCE[0]}" )

RESULT=0

for f in $(find . ${PRUNE_CMD} -path "*.git/*" -prune -o -name '*.c' -print -o -name '*.cpp' -print -o -name '*.hpp' -print -o -name '*.h' -print); do
	if [ "$DRYRUN" = true ] ; then
		astyle --options=${TOOLSDIR}/astylerc --preserve-date --dry-run $f | grep "Formatted"
		if [[ $? -eq 0 ]]
		then
			echo $f "bad formatting"
		RESULT=1
		fi

	else
		astyle --options=${TOOLSDIR}/astylerc --preserve-date $f | grep "Formatted"
	fi
done

if [[ $RESULT -eq 1 ]]
then
	echo 'Please run "make fix-style"'
fi

exit $RESULT
