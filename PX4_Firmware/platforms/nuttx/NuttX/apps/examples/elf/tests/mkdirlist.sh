#!/usr/bin/env bash

usage="Usage: %0 <fsimg-dir-path>"

dir=$1
if [ -z "$dir" ]; then
	echo "ERROR: Missing <fsimg-dir-path>"
	echo ""
	echo $usage
	exit 1
fi

if [ ! -d "$dir" ]; then
	echo "ERROR: Directory $dir does not exist"
	echo ""
	echo $usage
	exit 1
fi

echo "#ifndef __EXAMPLES_ELF_TESTS_DIRLIST_H"
echo "#define __EXAMPLES_ELF_TESTS_DIRLIST_H"
echo ""
echo "static const char *dirlist[] ="
echo "{"

for file in `ls $dir`; do
	echo "  \"$file\","
done

echo "  NULL"
echo "};"
echo ""
echo "#endif /* __EXAMPLES_ELF_TESTS_DIRLIST_H */"


