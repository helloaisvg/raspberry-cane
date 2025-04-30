import onnx

# 加载原始 ONNX 模型
model = onnx.load('best.onnx')

# 将模型转换为 Opset 15
target_opset = 15
converted_model = onnx.version_converter.convert_version(model, target_opset)

# 保存转换后的模型
onnx.save(converted_model, 'best_opset15.onnx')