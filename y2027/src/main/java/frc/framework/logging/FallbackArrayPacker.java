package frc.framework.logging;

public class FallbackArrayPacker implements Packer<Object[]> {
	@SuppressWarnings(
		"unchecked"
	) @Override
	public Class<Object[]> getDataType() {
		return (Class<Object[]>) Object.class.arrayType();
	}
	
	@Override
	public Object[] packFields(String key, PackHelper helper) {
		int length = helper.getValue(DataLogUtils.joinKey(key, "length"), Integer.class);
		Object[] result = new Object[length];
		
		for (int i = 0; i < length; i++) {
			result[i] = helper.getValue(DataLogUtils.joinKey(key, String.valueOf(i)), Object.class);
		}
		
		return result;
	}
	
	@Override
	public void unpackFields(Object[] value, String key, PackHelper packHelper) {
		packHelper.addUnpackedField(DataLogUtils.joinKey(key, "length"), value.length);
		
		for (int i = 0; i < value.length; i++) {
			packHelper.addUnpackedField(DataLogUtils.joinKey(key, String.valueOf(i)), value[i]);
		}
	}
}
