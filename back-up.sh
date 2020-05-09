now="$(date +"%d/%m/%Y:%H:%M")"
#echo $now
sudo git commit -a -m "Changes date:$now"
sudo git push origin master