now="$(date +"%d/%m/%Y:%H:%M")"
#echo $now
sudo git commit -a -m "JSG - Changes date: $now"
sudo git push origin master
