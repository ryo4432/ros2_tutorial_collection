## test command in colcon

### test all
```
colcon test
```

### test for specific packages
```
colcon test --packages-up-to [package name]
```

example:
```
$ colcon test --packages-up-to tutorial_test
Starting >>> tutorial_composition
Finished <<< tutorial_composition [2.88s]
Starting >>> tutorial_test
Finished <<< tutorial_test [5.33s]

Summary: 2 packages finished [8.44s]
```
Test target package is tutorial_test, though related packages are also included in test target.

### test for pattern match packages
```
colcon test --packages-select-regex [package name]
```

### test for build failed packages
```
colcon test --packages-select-build-failed
```

### show summary
```
colcon test-result
```

### show all result
```
colcon test-result --all
```


## link
https://colcon.readthedocs.io/en/released/reference/verb/test.html#