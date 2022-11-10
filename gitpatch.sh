f=BatchSplitBVHBuilder3.hpp

for h in $(cat /tmp/commits)
do
  h="${h//$'\r\n'/}"
  
  git checkout "$h" >&  /dev/null
  # git log -n 1
  #  >> ../gitloop/runlog.txt

  echo "$f  $h" `diff ../current/$f src/rt/bvh/BatchSplitBVHBuilder.hpp | wc`
done
