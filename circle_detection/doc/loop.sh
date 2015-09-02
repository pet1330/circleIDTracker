((i=0))
for (( c=1; c<=10; ((c=$c+1)) ))
do
	echo $i
	./circlecode-batch-change-inner-size.sh $i
	i=$(($i+1))
done	


# ((i=0))
# cat people.txt | while read line
# do
# 	echo $i
# 	./circlecode-batch-change-inner-size.sh $i
# 	i=$(($i+1))
# done