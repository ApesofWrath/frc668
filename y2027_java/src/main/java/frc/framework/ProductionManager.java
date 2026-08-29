package frc.framework;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;

public class ProductionManager {
    public ArrayList<Production> productions = new ArrayList<>();
    public HashMap<Object, Object> values = new HashMap<>();

    public void addProduction(Production prod) {
        productions.add(prod);
    }

    @SuppressWarnings("unchecked")
    public <T> T getValue(FieldReference<T> field) {
        if (values.containsKey(field)) {
            return (T)values.get(field);
        }
        return field.defaultValue;
    }

    public <T> void setValue(FieldReference<T> field, T value) {
        values.put(field, value);
    }

    public void resolve() {
        values.clear();

        productions.sort(new ProductionComparator());
        
        for (Production production : productions) {
            boolean isValid = true;

            for (Object key : values.keySet()) {
                if (values.containsKey(key)) isValid = false;
            }

            if (!isValid) continue;

            for (Entry<Object, Object> entry : production.values.entrySet()) {
                values.put(entry.getKey(), entry.getValue());
            }
        }
    }
}
