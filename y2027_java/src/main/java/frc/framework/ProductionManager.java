package frc.framework;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;

/**
 * Manages the evaluation of several productions.
 */
public class ProductionManager {
    public ArrayList<Production> productions = new ArrayList<>();
    public HashMap<Object, Object> values = new HashMap<>();

    /**
     * Add a production to influence the behavior of Executors
     * @param prod The production to add
     */
    public void addProduction(Production prod) {
        productions.add(prod);
    }

    /**
     * Return the value of field, falling back to the default value if no productions produce it
     * @param <T> The value type of the field
     * @param field The field reference to query for
     * @return
     */
    @SuppressWarnings("unchecked")
    public <T> T getValue(FieldReference<T> field) {
        if (values.containsKey(field)) {
            return (T)values.get(field);
        }
        return field.defaultValue;
    }

    /**
     * Apply the highest priority productions first, skipping productions that conflict with others.
     * @see Production
     */
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
