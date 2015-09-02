id=$1

convert -size 648x398 xc:white -bordercolor black -border 1 "$id.png" 

num_circle=5;
num_circle_per_row=3;
r_outer=90;
r_inner_max=35;
r_inner_min=35;
r_inner_del=$((($r_inner_max-$r_inner_min)/($num_circle-1)));

# Black circles
for i in $(seq 0 $(($num_circle-1)));do 
	convert "$id.png" -fill black -stroke black -draw "ellipse $((($i%$num_circle_per_row)*220+115+115*($i/$num_circle_per_row))),$((115+180*($i/$num_circle_per_row))) $r_outer,$r_outer 0,360"  "$id.png";
done

an=$(($1))
	
# White circles
for i in $(seq 0 $(($num_circle-1)));do 
	ax=$((an%4))
	an=$((an/4))
	ca=0
	sa=0
	if [ $(($ax%2)) == 0 ];
	then
		ca=$((2*($ax/2)-1))
	else
		sa=$((2*($ax/2)-1))
	fi
	r_inner=$(($r_inner_min+$r_inner_del*i));
	
	if [[ $i > 2 ]]; then
		# Shift
		xOffset=$(($i%$(($num_circle_per_row-1))))
		convert "$id.png" -stroke white -fill white -draw "ellipse $(($xOffset*220+115+115*($i/$num_circle_per_row)+$ca*20)),$(((115+180*($i/$num_circle_per_row))+$sa*20)) $r_inner,$r_inner 0,360" "$id.png";
	else
		convert "$id.png" -stroke white -fill white -draw "ellipse $((($i%$num_circle_per_row)*220+115+115*($i/$num_circle_per_row)+$ca*20)),$(((115+180*($i/$num_circle_per_row))+$sa*20)) $r_inner,$r_inner 0,360" "$id.png";
	fi
done
