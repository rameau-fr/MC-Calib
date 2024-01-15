docker run `
    -ti --rm `
    --volume="$(PWD):/home/MC-Calib" `
    --volume="D:\2023_Fall\EVENT\mc_calib\data\dataset:/home/MC-Calib/data" `
    bailool/mc-calib-prod