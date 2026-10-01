# Host tests

## Z80 core vs SingleStepTests

1. Fetch the corpus (MIT, ~290 MB zip) outside the repo, e.g. into `../lab/sst/`:
   `curl -L -o z80-sst.zip https://github.com/SingleStepTests/z80/archive/ebe1875d48f374bcfd4b505d8eb8ee751568b5f7.zip && unzip z80-sst.zip`
2. `python3 tests/host/sst_convert.py <dir>/v1 corpus.bin`
3. `cd tests/host && make && ./sst_run corpus.bin`

Expected: `pass 1604000 fail 0 (cycle mismatches 0)`. The run takes about 3 s.
