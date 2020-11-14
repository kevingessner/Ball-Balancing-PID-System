$pypath = "$($env:LOCALAPPDATA)\Programs\Python\Python38\python.exe"
echo $pypath
$pyver = "$(& $pypath --version)"
if ($pyver -eq "") {
	echo "Please install python3 from https://www.python.org/downloads/release/python-386/"
	pause
	return 3
}

echo "continuing with $pyver"
cd $PSScriptRoot
& $pypath -m venv venv
echo "installing"
& .\venv\Scripts\python.exe -m pip install -r requirements.txt --only-binary numpy
echo "done"