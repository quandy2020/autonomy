# Local model artifacts (gitignored)

Copy the lingbot_depth_trt student TensorRT engine here:

```bash
cp /path/to/lingbot_depth_trt/model/student_fp16.engine \
  autonomy/perception/fathom/model/student_fp16.engine
```

Expected profile: FP16, `image` `[1,3,480,640]`, `raw_depth` `[1,1,480,640]`,
`pred_depth` `[1,480,640]`.
