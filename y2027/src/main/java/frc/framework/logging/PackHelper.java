package frc.framework.logging;

import java.util.HashMap;
import java.util.HashSet;

/**
 * A helper class for writing packers and unpackers, contains a whole bunch of values and entries and provides methods
 * for updating the values, declaring entries, getting values, etc.
 */
public class PackHelper {
	private final HashMap<String, Object> data = new HashMap<>();
	private final HashSet<LogField> fields = new HashSet<>();
	
	/**
	 * Add a field to the fields list
	 *
	 * @param field The field to add
	 */
	public void addField(LogField field) {
		fields.add(field);
	}
	
	/**
	 * Add a unpacked field (a field that is a portion of a packed type)
	 *
	 * @param key   The key of the unpacked field
	 * @param value The value in the unpacked field
	 */
	public void addUnpackedField(String key, Object value) {
		String typeId = DataLogUtils.getTypeId(DataLogUtils.normalizeValue(value));
		
		if (typeId == null) {
			unpackFields(key, value, true);
			return;
		}
		
		data.put(key, value);
		fields.add(new LogField(key, typeId));
	}
	
	/**
	 * @return The value of the data field
	 */
	public HashMap<String, Object> getData() {
		return data;
	}
	
	/**
	 * @return The value of the entries field
	 */
	public HashSet<LogField> getFields() {
		return fields;
	}
	
	/**
	 * Read a value, packing it if need be
	 *
	 * @param key        The key to read from
	 * @param valueClass The reflection type of the value
	 * @param <T>        The desired value type
	 *
	 * @return The read type
	 */
	@SuppressWarnings(
		"unchecked"
	)
	public <T> T getValue(String key, Class<T> valueClass) {
		Object value = getValueUntyped(key);
		
		boolean isSpecialIntCase = value.getClass().getName().equals("java.lang.Long") && valueClass.getName()
			.equals("java.lang.Integer");
		
		if (!valueClass.isInstance(value) && !isSpecialIntCase) {
			throw new ClassCastException(
				"Cannot cast from " + value.getClass().getName() + " to " + valueClass.getName()
			);
		}
		
		if (isSpecialIntCase) {
			return (T) Integer.valueOf(value.toString());
		}
		
		return (T) value;
	}
	
	/**
	 * Read a value without type checking
	 *
	 * @param key The key to read from
	 *
	 * @return The read value
	 */
	public Object getValueUntyped(String key) {
		LogField field = null;
		
		for (LogField logField : fields) {
			if (logField.id().equals(key)) { field = logField; }
		}
		
		if (field == null) { throw new RuntimeException("Cannot find field " + key); }
		
		String fieldMetadata = field.metadata();
		
		if (fieldMetadata.equals("primitive") || fieldMetadata.isEmpty()) {
			return data.get(key);
		}
		
		String packerId = null;
		
		if (fieldMetadata.startsWith("packed::")) {
			packerId = fieldMetadata.substring(8);
		} else if (fieldMetadata.startsWith("nested::packed::")) {
			packerId = fieldMetadata.substring(16);
		}
		
		if (packerId == null) {
			throw new RuntimeException("Cannot find packer ID in " + fieldMetadata + " on key " + key);
		}
		
		Packer<?> packer = null;
		
		Packer.setupPackers();
		
		for (Packer<?> pkr : Packer.packers) {
			if (pkr.getId().equals(packerId)) {
				packer = pkr;
				break;
			}
		}
		
		if (packer == null) { throw new RuntimeException("Cannot find packer with ID " + packerId + " on key " + key); }
		
		return packer.packFields(key, this);
	}
	
	/**
	 * Set a value in the internal data table
	 *
	 * @param key   The key to store to
	 * @param value The value to store
	 */
	public void setValue(String key, Object value) {
		data.put(key, value);
	}
	
	/**
	 * Unpack a value under a given key
	 *
	 * @param key      The key to unpack
	 * @param value    The value to pack
	 * @param isNested Is this value to be unpacked under another unpacked value
	 */
	public void unpackFields(String key, Object value, boolean isNested) {
		for (Packer<?> packer : Packer.packers) {
			if (!packer.getDataType().isInstance(value)) { continue; }
			fields.add(new LogField(key, "boolean", (isNested ? "nested::packed::" : "packed::") + packer.getId()));
			packer.unpackFieldsUnsafe(value, key, this);
			
			return;
		}
		
		System.err.println("Cannot pack value " + key);
	}
}
