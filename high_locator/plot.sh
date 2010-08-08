#!/bin/bash
for i in $( ls | grep gsh ); do
    gnuplot $i
done

echo "set terminal png" > plotall.gsh
#set encoding iso_8859_5
echo "set output \"sygnal.png\"" >>plotall.gsh
echo "set grid" >>plotall.gsh
echo "set nokey" >>plotall.gsh
#set yrange [-300:300]
echo "plot" >> plotall.gsh 
for i in $( ls | grep dat ); do
    echo "\""$i"\" with lines, \\" >> plotall.gsh
done
