git add .
git commit -am "cleaned code + comments"
git push
dts devel build --push -u ai404

docker -H autobot pull ai404/dt-pure-pursuit:v1-arm32v7
dts duckiebot demo --demo_name lane_follow --package_name pure-pursuit --duckiebot_name autobot --image ai404/dt-pure-pursuit:v1-arm32v7