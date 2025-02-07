#!/bin/bash
function getArrItemIdx(){
local arr=$1
local item=$2
local index=1
for i in ${arr[*]}
do
  if [[ $item == $i ]]
    then
    echo $index
    return
  fi
  index=$(( $index + 1 ))
done
}
partitionsId=(1 2 3 4 5 6 7 9 10 12 13 14 15 16)
a=`getArrItemIdx "${partitionsId[*]}" 10`
echo $a