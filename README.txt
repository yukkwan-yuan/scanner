剛開始clone下來的時候，進入docker去catkin_make的時候，會顯示沒有權限
這是要進入docker去看UID，指令 echo $UID
這是UID會顯示1000
所以你要退出docker和當前文件夾之後，執行sudo chown -R $UID:1000 scanner/ 即可
然後編譯完成後，需要source compile_py36.sh 一次，docker即可正常運作

