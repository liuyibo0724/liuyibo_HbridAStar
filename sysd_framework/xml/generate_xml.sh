#!/usr/bin/zsh
if ! which xmllint; then
    echo "Error: Unable to find tool -- xmllint < sudo apt install libxml2-utils >"
    exit -1
fi 

set -o errexit

# add rti tools to current environment
export PATH=/opt/rti_connext_dds-6.0.1/bin:$PATH

repo_base=$(cd $(dirname "$0") && pwd)/../..
cd $repo_base/idl_generated


printf "\n-- generating xml files...\n"
for idl_file in $(find -name "*.idl"); do
    printf "\033[32mgenerating %s\n\033[0m" $idl_file
    rtiddsgen -convertToXml $idl_file -d $repo_base/sysd_framework/xml/
done

set +o errexit

cd $repo_base/sysd_framework/xml
printf "\n-- post processing xml files...\n"
for xml_file in ./*.xml; do
    xmllint --format $xml_file > $xml_file.format && mv $xml_file.format $xml_file

    printf "\033[32mpost processing %s\n\033[0m" $xml_file
    if ! grep -q -r '<type>' $xml_file; then
        sed -i -e 's/<types .*/<types>/g' -e '/<types/a\<type>' -e '/<\/types>/i\</type>' $xml_file
    fi
    include_lines=$(grep -r " <include file=" $xml_file)

    if [[ "" != ""$include_lines ]]; then
        echo "$include_lines" | while read line; do
            sed -i -e 's#'$line'#<!--'${line}'-->#g' $xml_file
        done
    fi
    for enum_name in `grep -r "<enum name=" $xml_file | awk -F \" '{print $2}'`; do
        sed -i -e 's/type="nonBasic" nonBasicTypeName="'$enum_name'"/type="uint32"/g' $xml_file
    done
    xmllint --format $xml_file > $xml_file.format && mv $xml_file.format $xml_file
done

printf "\n-- sorting xml files by include depth...\n"
function get_xml_include_depth() {
    file=$1
    # echo "get_xml_include_depth $file"
    grep_result=$(grep -r "<include file=" $file 2>/dev/null)
    if [[ "" == ""$grep_result ]]; then 
        return 0;
    fi
    max_depth=0
    for include_file in `echo "$grep_result" | awk -F \" '{print $2}'`; do
        get_xml_include_depth $include_file
        depth=$?
        if [[ $depth > $max_depth ]]; then
            max_depth=$depth
        fi
    done
    return `echo "$max_depth+1" | bc`
}
rm -r ./sorted/*.xml 
for xml_file in ./*.xml; do 
    get_xml_include_depth $xml_file
    depth=$?
    base_name=$(basename ${xml_file}) 
    echo "include depth: $depth $base_name"
    cp $xml_file ./sorted/${depth}_$base_name
done
