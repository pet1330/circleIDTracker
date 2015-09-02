((i=0))
cat people.txt | while read line
do
	echo $i
	./circlecode-batch-change-inner-size.sh $i
	i=$(($i+1))
done