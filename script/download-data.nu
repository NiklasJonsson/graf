def main [] {
    # nushell
    let data_root = "data"
    print $"Downloading DAO map data to ./($data_root)" 

    let files = [
        "dao-scen",
        "dao-map",
    ]

    $files | par-each { |file|
        let url = $"https://movingai.com/benchmarks/dao/($file).zip"
        let dst = $"($data_root)/($file).zip"
        let dir = $"($data_root)/($file)"

        print $"Downloading ($url) to ($dst)"
        http get $url | save $dst

        print $"Unzipping ($dst) to ($dir)"
        mkdir $dir
        tar -xf $dst -C $dir
    }

    print "Cleaning up zip files"
    rm data/*.zip
}