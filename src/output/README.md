# AVRO Output

## 1. Bash: JSON -> .h

```bash
./build/bin/avrogencpp  -i src/output/person.json -o src/output/person.h -n moss
./build/bin/avrogencpp  -i src/output/tl.json -o src/output/tl.h -n moss
```

## 2. Manual: JSON -> std::string

Copy the schema string into the `schema.h` file.
